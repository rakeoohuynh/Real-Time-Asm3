Ừ, với README hiện tại thì **code của bạn đã được thiết kế để chạy trong QNX Momentics**, dù bạn viết code bằng VS Code trước rồi mới paste/import sang Momentics. Việc viết bằng VS Code **không có vấn đề gì** miễn là project cuối cùng được build bằng QNX toolchain (`qcc`) trong Momentics.

Quan trọng hơn: **bạn không cần VM3** cho phase hiện tại. README đã xác định rõ `vm1` + `vm2` là đủ. 

---

# 1. Hiện tại chương trình của bạn chạy như thế nào?

Kiến trúc hiện tại là:

```text
                    VM1
        ┌──────────────────────────┐
        │                          │
        │     Central Controller   │
        │            cc            │
        │             ▲            │
        │             │ local      │
        │             │ IPC        │
        │             │             │
        │      Local Controller I1 │
        │             │             │
        │       ┌─────┴─────┐      │
        │       │           │      │
        │   Phase Ctrl   Signal    │
        │   (main)       Output    │
        │       │           │      │
        │       └───────────┘      │
        └──────────────────────────┘
                     ▲
                     │ Qnet
                     │
        ┌──────────────────────────┐
        │           VM2            │
        │                          │
        │      Local Controller I2 │
        │             │            │
        │       ┌─────┴─────┐      │
        │       │           │      │
        │   Phase Ctrl   Signal    │
        │   (main)       Output    │
        │                          │
        └──────────────────────────┘
```

README của bạn xác định:

* `vm1`: chạy `lc I1` + `cc`
* `vm2`: chạy `lc I2`
* I1 và I2 là **hai QNX process độc lập**
* CC chạy trên vm1
* I2 gửi status sang CC qua Qnet. 

Đây là một architecture khá hợp lý cho phase hiện tại.

---

# 2. Các file hiện tại dùng để làm gì?

README của bạn đã định nghĩa structure như sau: 

```text
TrafficControllerSystem/
│
├── README.md
├── Makefile
│
├── common/
│   ├── protocol.h
│   ├── protocol.c
│   └── timing.h
│
├── lc/
│   ├── lc_main.c
│   ├── phase_controller.h
│   ├── phase_controller.c
│   ├── signal_output.h
│   ├── signal_output.c
│   ├── status_reporting.h
│   └── status_reporting.c
│
└── cc/
    └── cc_main.c
```

### `lc/lc_main.c`

Đây là **entry point của Local Controller**.

Một executable:

```bash
./lc I1
```

sẽ trở thành Local Controller của I1.

Còn:

```bash
./lc I2
```

sẽ trở thành Local Controller của I2.

Nên **không cần viết `lc_I1.c` và `lc_I2.c` riêng**.

Cùng một executable `lc`, chỉ khác:

```text
argv[1] = I1
```

hoặc:

```text
argv[1] = I2
```

README cũng xác nhận I1/I2 dùng cùng architecture và `argv[1]` quyết định intersection. 

---

# 3. `lc_main.c` của bạn đang làm những gì?

Code bạn gửi rất rõ.

### Bước 1 — đọc I1/I2

```c
if (strcmp(argv[1], "I1") == 0) {
    id = INTERSECTION_I1;
} else if (strcmp(argv[1], "I2") == 0) {
    id = INTERSECTION_I2;
}
```

Nghĩa là:

```bash
./lc I1
```

→ LC này là I1.

```bash
./lc I2
```

→ LC này là I2.

---

### Bước 2 — tạo channel cho Signal Output

```c
int chid_signal = ChannelCreate(0);
```

Channel này thuộc process hiện tại.

Sau đó `Signal_Output_Task` sẽ `MsgReceive()` trên channel này.

---

### Bước 3 — tạo channel cho Status Reporting

```c
int chid_status = ChannelCreate(0);
```

Channel thứ hai dành cho `Status_Reporting_Task`.

---

### Bước 4 — tạo 2 pthread

```c
pthread_create(&sig_thread, NULL, signal_output_task, &sig_args);
```

và:

```c
pthread_create(&stat_thread, NULL, status_reporting_task, &stat_args);
```

Vì vậy một LC process có:

```text
LC process
│
├── Main thread
│      └── Phase_Controller_Task
│
├── pthread 1
│      └── Signal_Output_Task
│
└── pthread 2
       └── Status_Reporting_Task
```

Đúng với architecture trong README. 

---

# 4. Phase Controller làm gì?

`phase_controller.c` là **bộ não của intersection**.

Nó:

1. tạo periodic timer;
2. nhận timer pulse;
3. đếm elapsed time;
4. kiểm tra phase hiện tại đã hết thời gian chưa;
5. chuyển sang phase tiếp theo;
6. gửi `SET_VEHICLE` đến Signal Output;
7. gửi `STATUS` đến Status Reporting.

State machine của bạn là:

```text
NS_GREEN  45s
    ↓
NS_YELLOW 3s
    ↓
ALL_RED 2s
    ↓
EW_GREEN 45s
    ↓
EW_YELLOW 3s
    ↓
ALL_RED 2s
    ↓
NS_GREEN
```

Đây là state machine bạn đã map từ Assessment 2. 

Và default demo scale đang làm cycle nhanh hơn để demo.

---

# 5. Signal Output Task làm gì?

Nó giả lập/đại diện cho **physical traffic-light signal heads**.

Phase Controller nói:

```text
SET_VEHICLE:
NS = GREEN
EW = RED
```

thì Signal Output Task nhận message:

```text
MsgReceive()
```

kiểm tra safety:

```text
NS GREEN + EW GREEN ?
```

Nếu conflict:

```text
NACK
```

Nếu hợp lệ:

```text
ACK
```

Sau đó in:

```text
SIGNAL HEADS -> NS:GREEN EW:RED
```

README xác nhận Signal Output có một channel riêng và dùng blocking `MsgReceive`/`MsgReply`. 

---

# 6. Status Reporting Task làm gì?

Task này **không điều khiển traffic light**.

Nó chỉ lấy status từ Phase Controller rồi gửi về CC.

Flow:

```text
Phase Controller
       │
       │ STATUS pulse
       ▼
Status Reporting Task
       │
       │ STATUS_UPDATE pulse
       ▼
Central Controller
```

Điểm quan trọng là nó dùng **pulse**, không phải blocking `MsgSend`.

Vì vậy CC chết/mất kết nối thì:

```text
Phase Controller
       ↓
traffic lights
       ↓
vẫn chạy
```

không bị CC làm block.

Đây chính là autonomy requirement bạn đã ghi trong README. 

---

# 7. Central Controller `cc` làm gì?

`cc` chỉ:

```text
receive status
      ↓
display status
```

Nó **không điều khiển traffic lights** trong phase này.

Ví dụ:

```text
[CC] I1: NS_GREEN -> NS_YELLOW
[CC] I2: EW_GREEN -> EW_YELLOW
```

Nó tồn tại chủ yếu để chứng minh:

```text
LC → CC
```

IPC hoạt động.

README xác định CC là process riêng và sử dụng global named channel. 

---

# 8. Vậy chạy thực tế trong Momentics như thế nào?

Bạn **không cần chạy code trực tiếp bằng VS Code**.

VS Code chỉ có thể coi là editor.

Flow của bạn nên là:

```text
VS Code
   │
   │ write/edit C
   ▼
Project files
   │
   │ copy/import
   ▼
QNX Momentics
   │
   │ qcc / Make
   ▼
bin/lc
bin/cc
   │
   ├───────────────┐
   ▼               ▼
 VM1             VM2
   │               │
 lc I1             lc I2
 cc
```

README của bạn cũng hướng dẫn import project vào Momentics, sau đó `Project → Build Project`, tạo `bin/lc` và `bin/cc`. 

---

# 9. Bạn cần paste từng file vào Momentics không?

**Không nên paste từng file manually.**

Tốt nhất là đưa nguyên project:

```text
TrafficControllerSystem/
```

vào workspace của Momentics.

Sau đó:

```text
File
 → Import
 → General
 → Existing Projects into Workspace
```

hoặc tạo QNX C Project/Makefile project trỏ đến folder đó.

README của bạn đã ghi đúng workflow này. 

---

# 10. Cực kỳ quan trọng: `ChannelCreate()` có tự lấy CID không?

### Có.

Đoạn này:

```c
int chid_signal = ChannelCreate(0);
```

**tự động tạo channel và trả về Channel ID (`chid`)**.

Ví dụ:

```text
ChannelCreate()
      ↓
chid = 1
```

Bạn **không tự nhập CID bằng tay**.

Tương tự:

```c
int chid_status = ChannelCreate(0);
```

có thể trở thành:

```text
chid_signal = 1
chid_status = 2
```

Nhưng số thực tế **không nên hard-code**, vì QNX sẽ cấp channel ID lúc runtime.

---

# 11. Còn PID thì sao?

PID cũng **tự động**.

Khi bạn chạy:

```bash
./lc I1
```

QNX tạo process.

Ví dụ:

```text
PID = 12345
```

Bạn lấy PID bằng:

```c
getpid()
```

Code của bạn đã làm điều này ở đây:

```c
coid = ConnectAttach(
    ND_LOCAL_NODE,
    getpid(),
    chid_signal,
    _NTO_SIDE_CHANNEL,
    0
);
```

Nên:

```text
PID → tự lấy bằng getpid()
CID → tự lấy từ ChannelCreate()
```

**Không cần add tay.**

---

# 12. Nhưng có một thứ bạn PHẢI cấu hình tay: node name

Đây là phần dễ nhầm nhất.

Bạn có:

```text
PID
CID
COID
Node ID
```

Chúng không giống nhau.

### PID

Process ID.

QNX tự cấp:

```c
getpid()
```

---

### CID

Channel ID.

QNX tự cấp:

```c
ChannelCreate()
```

---

### COID

Connection ID.

Được tạo khi bạn:

```c
ConnectAttach(...)
```

Ví dụ code của bạn:

```c
ConnectAttach(
    ND_LOCAL_NODE,
    getpid(),
    chid_signal,
    _NTO_SIDE_CHANNEL,
    0
);
```

Cũng **không cần tự nhập COID**.

---

### Node

Đây mới là thứ liên quan đến:

```text
vm1
vm2
```

và Qnet.

Ví dụ I2:

```bash
./lc I2 /net/vm1/dev/name/global/traffic_cc_status
```

Ở đây:

```text
/net/vm1/
```

là đường dẫn tới **node vm1**.

Cái này phải đúng với QNX network/node naming của bạn.

README cũng nói phải thay `vm1` nếu node thực tế có tên khác. 

---

# 13. Tóm tắt CID/PID/COID cho bạn

Bạn có thể nhớ bảng này:

| Thứ          | Ai tạo?                | Code hiện tại            |
| ------------ | ---------------------- | ------------------------ |
| PID          | QNX                    | `getpid()`               |
| CID          | QNX                    | `ChannelCreate()`        |
| COID         | QNX                    | `ConnectAttach()`        |
| Node         | Bạn/QNX network config | `vm1`, `vm2`, `/net/...` |
| Channel name | Programmer             | `traffic_cc_status`      |

Vậy **không có chuyện bạn phải ngồi xem CID rồi hard-code nó vào code**.

---

# 14. Vậy code hiện tại có đáp ứng việc "code ở VS Code, chạy ở Momentics" không?

### Có — về nguyên tắc là có.

Code của bạn là C/QNX code, không phụ thuộc VS Code.

Ví dụ:

```c
#include <sys/neutrino.h>
```

```c
ChannelCreate()
```

```c
MsgReceive()
```

```c
MsgSend()
```

```c
MsgReply()
```

```c
timer_create()
```

đều cần QNX environment/toolchain để compile.

Cho nên:

```text
VS Code
= nơi viết code

Momentics/QNX SDP
= nơi build bằng QNX toolchain + deploy/run trên QNX
```

Đây hoàn toàn là workflow hợp lý.

README của bạn cũng xác định build bằng:

```bash
make
```

để tạo:

```text
bin/lc
bin/cc
```

và sau đó deploy chúng lên VM. 

---

# 15. Nhưng có một điều mình muốn bạn kiểm tra trước khi paste sang Momentics

**Đừng chỉ copy `lc_main.c`.**

Bạn cần nguyên dependency tree:

```text
lc_main.c
   │
   ├── protocol.h
   ├── phase_controller.h
   ├── signal_output.h
   └── status_reporting.h
```

và các `.c` tương ứng:

```text
phase_controller.c
signal_output.c
status_reporting.c
protocol.c
```

plus:

```text
timing.h
Makefile
```

Nếu chỉ paste:

```text
lc_main.c
```

thì chắc chắn chưa đủ để build.

---

# 16. Cách chạy demo hiện tại — rất cụ thể

Sau khi build thành công:

## VM1

Mở terminal của `vm1`.

### Terminal 1 — CC

```bash
./cc
```

### Terminal 2 — I1

```bash
./lc I1
```

---

## VM2

Mở terminal của `vm2`:

```bash
./lc I2 /net/vm1/dev/name/global/traffic_cc_status
```

Đây chính xác là mapping README hiện tại. 

---

# 17. Output bạn nên thấy

I1 kiểu:

```text
QNX Traffic Light Local Controller - I1 (fixed_time mode)

Initial phase: NS_GREEN
SIGNAL HEADS -> NS:GREEN EW:RED

[I1] NS_GREEN -> NS_YELLOW
SIGNAL HEADS -> NS:YELLOW EW:RED

[I1] NS_YELLOW -> ALL_RED
SIGNAL HEADS -> NS:RED EW:RED

[I1] ALL_RED -> EW_GREEN
SIGNAL HEADS -> NS:RED EW:GREEN
```

I2 tương tự:

```text
[I2] NS_GREEN -> NS_YELLOW
...
```

CC sẽ nhận status từ cả hai:

```text
[CC] I1 ...
[CC] I2 ...
```

Test plan hiện tại của README cũng yêu cầu kiểm tra startup, fixed-time transition, complete cycle, multiple cycles, concurrent I1/I2 và cross-node IPC. 

---

# 18. Một điểm rất đáng chú ý trong code hiện tại

Bạn đang dùng:

```c
ConnectAttach(
    ND_LOCAL_NODE,
    getpid(),
    chid_signal,
    ...
);
```

cho **same-process/local communication**.

Đây là chủ ý đúng với architecture:

```text
Phase Controller
      ↓
Signal Output
```

vì cả hai đều nằm trong **cùng một LC process**.

Còn:

```text
I2
 ↓
Qnet
 ↓
vm1
 ↓
CC
```

thì dùng:

```text
/net/vm1/dev/name/global/traffic_cc_status
```

để tìm named attach point.

README xác định rõ local PID/CID connection cho các task trong cùng process và named global connection cho CC qua Qnet. 

---

# 19. Một vấn đề mình sẽ kiểm tra kỹ trước khi assessment

Có một điểm trong README cần **phân biệt giữa "thiết kế đúng" và "code thực sự compile/run đúng"**.

README nói:

> `lc` creates channels before threads start, so both chids are valid...

Điều đó tốt.

Nhưng việc:

```c
pthread_create(...)
```

truyền:

```c
&sig_args
&stat_args
```

cũng cần đảm bảo lifetime của các struct này an toàn trong suốt thời gian thread dùng chúng.

Trong code hiện tại:

```c
signal_output_args_t sig_args = { id, chid_signal };
status_reporting_args_t stat_args = { id, chid_status, cc_name };
```

chúng nằm trên stack của `main()`, nhưng `main()` không return ngay; nó block trong:

```c
phase_controller_run(...)
```

nên **trong kiến trúc hiện tại chúng vẫn còn tồn tại khi worker threads chạy**. Đây không nhất thiết là bug.

---

# 20. Kết luận cho trường hợp của bạn

### Bạn hiện tại KHÔNG cần VM3.

```text
VM1
 ├── cc
 └── lc I1

VM2
 └── lc I2
```

là đúng theo README hiện tại. 

### Bạn KHÔNG cần tự nhập PID/CID.

```text
PID → getpid()
CID → ChannelCreate()
COID → ConnectAttach()
```

đều được QNX/runtime xử lý.

### Bạn chỉ cần đảm bảo Qnet/node name đúng.

Đặc biệt:

```bash
/net/vm1/dev/name/global/traffic_cc_status
```

phải match node name thực tế của VM1.

### Viết bằng VS Code rồi chạy bằng Momentics: **OK.**

Miễn là:

```text
VS Code
  ↓
copy/import full project
  ↓
QNX Momentics
  ↓
qcc / make
  ↓
bin/lc + bin/cc
  ↓
VM1 + VM2
```

### Và quan trọng nhất:

**Đừng paste từng file vào Momentics một cách thủ công nếu không cần.** Hãy đưa cả `TrafficControllerSystem` project vào Momentics, để giữ nguyên `Makefile`, `common/`, `lc/`, `cc/` và dependencies.

Nếu mục tiêu tiếp theo của bạn là **thực sự đem project hiện tại từ VS Code sang Momentics và chạy lần đầu**, mình khuyên làm theo thứ tự: **kiểm tra `Makefile` → import project → configure x86_64 target → build → connect vm1/vm2 → deploy → chạy `cc` → `lc I1` → `lc I2` → test Qnet**.
