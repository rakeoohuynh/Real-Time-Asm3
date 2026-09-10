# Bridged Qnet Multi-PC Testing

This document is a **mandatory testing procedure**, separate from the application
itself: it proves the existing `lc`/`cc` binaries communicate over **real Qnet,
across three physically separate computers, over a bridged network** — not
merely between two VMs nested on one laptop (which is all we'd verified before
this phase).

**Nothing in `common/`, `lc/`, or `cc/` changes for this document.** If a step
below fails, the fix is a network/VM/QNX-configuration fix, not a code fix —
see §6.

---

## 0. Team topology for this test

| PC | Owner | Host OS | Hypervisor | QNX target hostname | Role in §4 (this app) |
|---|---|---|---|---|---|
| **PC1** | Truc (you) | macOS | UTM, with QNX nested inside a UTM/Ubuntu ARM64 VM, inside QEMU (per the install guide) | `vm1` | `cc` + `lc I1` |
| **PC2** | Teammate | Linux | VirtualBox, QNX target VM directly | `vm2` | `lc I2` |
| **PC3** | Teammate | Windows | VirtualBox, QNX target VM directly | `vm3` | network-verification only in this phase (§4.1–§4.6); optional smoke test in §4.7 |

PC2 and PC3 are **single-layer** virtualization (VirtualBox running the QNX
target VM directly) — simpler than PC1's setup, which has **two layers**
(UTM's own Ubuntu VM must itself be bridged, and then QEMU's QNX target inside
that Ubuntu VM must *also* be bridged onto Ubuntu's now-real network interface).
§2 below covers each.

This phase's application only has two roles (`I1`, `I2`), so `vm3` doesn't run
the real deliverable — it exists purely so the network/Qnet layer can be proven
working across **all three** physical machines, per the mandatory requirement
that this not merely be a two-VM/two-PC result.

---

## 1. Before touching any VM: the WiFi risk

You're testing over shared WiFi. Flag this now, because it's the single most
common way this entire exercise silently fails with **zero application-level
symptoms to debug**:

- **AP/client isolation.** Many campus and public WiFi networks explicitly
  block client-to-client traffic for security, regardless of how correctly
  every VM is configured. §4.1 (Physical PC connectivity) is designed to catch
  this in the first 2 minutes, before you invest time in VM-level bridging.
- **802.1X / MAC-based admission control.** Enterprise WiFi (this very likely
  includes an RMIT campus network) often only allows traffic from *registered*
  device MAC addresses. A bridged VM presents a **new, unregistered virtual
  MAC** to the network — many such networks will silently drop its traffic
  even though the physical laptop's own MAC works fine.
- **WiFi bridging is inherently less reliable than wired bridging**, independent
  of any network policy: many WiFi drivers/APs don't cleanly support multiple
  MAC addresses being bridged through one radio (this is a hardware/driver
  limitation, not a QNX or configuration issue).

**If §4.1 fails and you've confirmed it's not a simple misconfiguration**,
the pragmatic fix that still satisfies "bridged, not NAT/host-only" is to
switch physical medium, not networking mode:
- Connect all three laptops to a single cheap **unmanaged Ethernet switch**
  (no internet uplink needed — it only needs to relay frames between the three
  laptops) via their Ethernet ports/USB-Ethernet adapters, or
- Use one laptop's personal **WiFi hotspot** as the shared network instead of
  campus WiFi.

Either still uses genuine Bridged Adapter/Bridged Network mode in every VM —
it just avoids a physical network you don't control and that may be actively
designed to block exactly this kind of traffic.

---

## 2. Bridged network configuration per platform

### 2.1 PC2 / PC3 — VirtualBox (Linux / Windows), single layer

1. Shut down the QNX target VM.
2. VirtualBox Manager → select the VM → **Settings → Network → Adapter 1**.
3. **Attached to:** `Bridged Adapter`.
4. **Name:** select the laptop's real, currently-active adapter (the WiFi
   adapter you're using — on Windows this list can be long and confusing;
   match it against `ipconfig`'s active adapter, not a virtual/loopback one).
5. Expand **Advanced** → **Promiscuous Mode: Allow All**. This is not optional
   for this test — VirtualBox's default ("Deny") blocks the kind of
   broadcast/multicast-like frames Qnet's node discovery relies on between
   bridged VMs, and is a very common reason bridged VMs can reach the internet
   fine but never see each other.
6. Start the VM.

### 2.2 PC1 — macOS + UTM (nested Ubuntu) + QEMU (nested QNX target)

Two layers must both be bridged, or the QNX target's traffic never reaches
the physical network at all.

**Layer A — UTM's own Ubuntu VM:**
1. Shut down the Ubuntu VM.
2. UTM → select `ubuntu-qnx-dev` → Edit (pencil) → **Network**.
3. **Network Mode:** `Bridged (Advanced)`.
4. **Bridged Interface:** the Mac's active adapter (check **System Settings →
   Network** for which one currently shows "Connected" — Wi-Fi in this case).
5. Save, start the VM.
6. Confirm Ubuntu got a real LAN IP:
   ```bash
   ip addr show enp0s1
   ```
   It should be in the **same subnet** as the Mac's own WiFi IP (compare
   against `ifconfig en0` in a **macOS Terminal**). A `169.254.x.x`
   (link-local) address, or no address at all, means DHCP failed on the
   bridged interface — almost always the WiFi risk from §1, not a UTM bug.

**Layer B — QEMU's QNX target, bridged onto Ubuntu's now-real interface:**

Ubuntu itself needs a Linux bridge that QEMU can attach the QNX target's NIC
to. Run in the **Ubuntu VM Terminal**:
```bash
sudo apt install -y bridge-utils qemu-utils

# Create the bridge and enslave Ubuntu's real interface to it
sudo ip link add name br0 type bridge
sudo ip link set enp0s1 master br0
sudo ip link set enp0s1 up
sudo ip link set br0 up

# Ubuntu's own IP now belongs on br0, not enp0s1
sudo dhclient br0

# Let QEMU's unprivileged bridge helper use br0
echo "allow br0" | sudo tee /etc/qemu/bridge.conf
sudo chmod u+s /usr/lib/qemu/qemu-bridge-helper
```
**Note:** if `dhclient br0` doesn't pick up an address within ~15s, re-check
§1 before troubleshooting the bridge itself — this is the exact symptom of
DHCP/admission-control rejecting the bridge's presence on the LAN.

Launch the QNX target with a bridged NIC **first** on the command line (so it
becomes the e1000 unit that `-d e1000` in `ifs_start.custom` binds to), keeping
the old `user`-mode NIC only as an SSH fallback:
```bash
cd ~/qnx-vms/vm1
qemu-system-x86_64 -M pc -accel tcg -smp 1 -m 1024 \
  -drive file=output/disk-qemu,if=ide,format=raw \
  -nic bridge,br=br0,model=e1000,mac=52:54:00:12:34:01 \
  -nic user,model=e1000,hostfwd=tcp::2222-:22 \
  -serial mon:stdio -display none
```

This bridged setup is **additional** to, not a replacement for, the
`mcast`/`listen`/`connect` socket networking from earlier in this project —
that configuration still works for same-Mac vm1↔vm2 development testing. Use
*this* configuration specifically for the cross-PC test in this document, then
switch back afterward (§7).

---

## 3. Mandatory verification order

Run every step below **for all three pairwise combinations**
(PC1↔PC2, PC1↔PC3, PC2↔PC3) before moving to the next step. Do not skip ahead
to running `lc`/`cc` — a failure two steps earlier will look identical to an
application bug if you do.

### 3.1 Physical PC connectivity
On each **host OS** (macOS Terminal / Linux terminal / Windows PowerShell —
NOT inside any VM yet), find each machine's own LAN IP and ping the others:
```bash
# macOS
ifconfig en0 | grep "inet "
# Linux
ip addr show <your wifi iface>
# Windows (PowerShell)
ipconfig
```
Then from each machine: `ping <other machine's IP>` (all three pairs).
**All three pings must succeed** before proceeding. If any fail, stop — this
is the §1 WiFi risk, not a VM problem.

### 3.2 VM network connectivity
Inside each **guest OS** hosting a QNX target (Ubuntu for PC1; the VirtualBox
host doesn't have a separate guest OS layer for PC2/PC3 — go straight to the
QNX target console there):
- **PC1 (Ubuntu):** `ip addr show br0` — confirm a real LAN IP, then
  `ping <PC2 host IP>` and `ping <PC3 host IP>` from inside Ubuntu.
- **PC2/PC3:** the QNX target itself is the "VM" here — see §3.3, since QNX's
  own IP (if any) is checked there. If VirtualBox reports the bridged adapter
  as "Cable Connected" in the VM's status bar, that's the PC2/PC3 equivalent
  check at this layer.

### 3.3 QNX node identity
On **each** QNX target console:
```bash
uname -n
```
Confirm the three targets report **three different names** (`vm1`, `vm2`,
`vm3`) — a name collision will cause confusing, silent failures in every step
after this one. If two show the same name, one target's image was built with
the wrong `--hostname` to `mkqnximage` — rebuild it with the correct one.

(Optional, only if you need the raw IP-level sanity check and your image has
`ifconfig`/a DHCP client: `ifconfig e1000` / assign a static IP as a pure
diagnostic — Qnet itself does **not** require this, see §4.)

### 3.4 Qnet connectivity
On each QNX target:
```bash
pidin ar | grep io-pkt
```
Expected on **all three**: `io-pkt-v6-hc -U 33:33 -d e1000 -p qnet`. If it's
missing on PC2 or PC3, their `ifs_start.custom`/`post_start.custom` snippets
need to be copied from PC1's, exactly as done when vm2 was first set up (see
the original install guide, Section 9 Step 2) — then rebuild that target's
image.

### 3.5 Remote `/net` visibility
On each QNX target:
```bash
ls /net
```
Expected on **PC1's target**: `vm2` and `vm3` (not just `vm1`).
Expected on **PC2's target**: `vm1` and `vm3`.
Expected on **PC3's target**: `vm1` and `vm2`.

This is the first genuinely cross-PC-specific proof — everything before this
point could theoretically pass even if the bridge only worked between two of
the three machines. If any target's `/net` is missing one of the other two
names, that specific pair's bridge/WiFi path is the problem — isolate it by
re-running §3.1–§3.2 for just that pair.

### 3.6 QNX named service visibility
This is the step that specifically proves **native QNX IPC** (not just raw
Qnet node discovery) works cross-PC, independent of our application. On
**PC1's** QNX target, start a throwaway attach point:
```bash
# On PC1's QNX target
mkdir -p /tmp/nettest 2>/dev/null
# (any resource manager works as a smoke test; reuse cc's own attach mechanism)
```
Simplest concrete test using this project's own binary: start `cc` on PC1
now (it attaches a local named channel at startup):
```bash
cd /tmp && ./cc &
```
Then, from **PC2's and PC3's** QNX targets, confirm the attach point is
independently visible over Qnet (this only checks filesystem-level visibility,
not the application protocol):
```bash
ls -la /net/vm1/dev/name/local/traffic_cc_status
```
Both PC2 and PC3 must see this entry. If PC2 sees it but PC3 doesn't (or vice
versa), that pair's path is broken at the Qnet/bridge layer specifically for
named-service traffic — re-check §3.4–§3.5 for that pair. Stop `cc` afterward
(`kill %1`) before §4, so its startup state is fresh for the real test.

### 3.7 Current application
Only after §3.1–§3.6 all pass for every pair, run the real deliverable:

```text
PC1: ./cc            (Terminal A)
PC1: ./lc I1          (Terminal B)
PC2: ./lc I2 /net/vm1/dev/name/local/traffic_cc_status
```
Expected output is exactly as described in `README.md` §8/§3 — I1 and I2 each
cycle independently, and PC1's `cc` console shows `STATUS_UPDATE` lines from
**both** `I1` (local, same PC) and `I2` (now genuinely cross-PC, over the
bridged network, not the nested QEMU virtual link used in earlier testing).

**Optional PC3 smoke test** (not the real deliverable, just extra proof this
isn't a PC1↔PC2-only result): on PC3's QNX target, confirm it can independently
reach PC1's `cc` at the filesystem level while the above is running:
```bash
ls -la /net/vm1/dev/name/local/traffic_cc_status
```
(A full third `lc` role isn't defined by this phase's application, so this
step stays at the filesystem/Qnet level rather than running a conflicting
duplicate `I1`/`I2` instance.)

---

## 4. Cross-computer proof checklist

At minimum, capture evidence (terminal screenshots/output) for:

- [ ] PC1 ↔ PC2: §3.1–§3.6 pass; §3.7 shows `I1`(PC1)/`I2`(PC2)/`cc`(PC1)
      working together, with `cc` receiving `I2`'s updates cross-PC.
- [ ] PC1 ↔ PC3: §3.1–§3.6 pass (PC3's `ls /net` shows `vm1`; PC3 sees PC1's
      `traffic_cc_status` attach point).
- [ ] PC2 ↔ PC3: §3.1–§3.6 pass (PC2's and PC3's `/net` each show the other).

All three pairs passing — not just PC1↔PC2 — is what proves this isn't merely
a same-physical-computer result.

---

## 5. Troubleshooting: which layer is actually broken?

**Do not modify `lc`/`cc` source in response to a failure here.** Work top-down
through the layers below — the failing layer is whichever one is the *last*
step that still passed:

| Symptom | Likely layer | Where to look |
|---|---|---|
| §3.1 ping fails between two hosts | Physical network | §1 (AP isolation / 802.1X) — try the Ethernet-switch or hotspot fallback |
| §3.1 passes, §3.2 VM has no/link-local IP | VM bridge config | Re-check §2 for that platform; on PC1, re-check `dhclient br0` |
| §3.2 passes, §3.3 shows duplicate hostnames | QNX image config | Rebuild the offending target with the correct `--hostname` |
| §3.3 passes, §3.4 missing `-p qnet` | QNX image snippets | Re-copy `ifs_*.custom`/`post_start.custom`, rebuild that image |
| §3.4 passes, §3.5 `/net` missing one peer | Bridge/Qnet for that specific pair | Re-test §3.1–§3.2 for just that pair; check VirtualBox Promiscuous Mode (§2.1 step 5) |
| §3.5 passes, §3.6 named service not visible | Unlikely if §3.5 passed — re-run §3.6, confirm `cc` was actually running on PC1 at the time |
| §3.1–§3.6 all pass, §3.7 application fails | **Only now** consider the application — check the `lc I2` argv path matches PC1's actual node name |

If §3.7 is where it breaks, it's the one case where the application/its
arguments are worth double-checking (e.g., a typo'd node name) — but only
after every earlier layer is independently confirmed working.

---

## 6. Reverting to local (same-PC) dev networking

Once this test is documented, PC1 can go back to the `mcast`/`listen`-`connect`
QEMU socket networking (see main `README.md`) for day-to-day development —
that configuration is simpler and doesn't depend on WiFi/campus network policy
at all, since it never leaves the Mac.
