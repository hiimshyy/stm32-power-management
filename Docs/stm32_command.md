# Lệnh gửi đến STM32 thông qua cổng serial

Tất cả lệnh được gửi đến STM32 thông qua cổng serial đều có định dạng JSON.

Về phản hồi, STM32 sẽ gửi phản hồi ACK đến ứng dụng sau khi nhận được lệnh và thực hiện lệnh thành công. Còn nếu lệnh thực hiện thất bại, STM32 sẽ gửi lỗi đến ứng dụng.
Chi tiết: [STM32 response ACK](stm32_response.md#3-phản-hồi-ack)

## 1. Định dạng tin nhắn

- Tất cả tin nhắn bắt đầu với ký tự `>`
- Tất cả tin nhắn kết thúc với `\r\n` (carriage return + line feed)
- Nội dung tin nhắn ở định dạng JSON

Ví dụ lệnh:
```
>{"type":0,"data":{"current":500}}\r\n
```

Cấu trúc JSON:
```json
{
  "id": <id>,
  "type": <cmd_type>,
  "data": <cmd_data>
}
```

### id

- Kiểu dữ liệu: string
- Mô tả: ID của lệnh

### cmd_type

| Loại | Kiểu dữ liệu | Mô tả                      |
|------|--------------|----------------------------|
| 0    | uint8        | Cấu hình sạc pin           |
| 1    | uint8        | Cấu hình xả pin            |
| 2    | uint8        | Cấu hình động cơ nâng hạ   |
| 3    | uint8        | Cấu hình động cơ di chuyển |

### cmd_data

Tùy theo `cmd_type` mà `cmd_data` có nội dung khác nhau.

## 2. Cmd Data

### 2.1. Cấu hình sạc pin (cmd_type = 0)

| Tham số        | Kiểu dữ liệu | Mô tả                                |
|----------------|--------------|--------------------------------------|
| current_limit  | uint16       | Dòng điện tối đa                     |
| enable         | uint8        | Cho phép sạc pin (0: tắt, 1: bật)    |

Ví dụ:
```json
>{"id":"abc","type":0,"data":{"current_limit":123,"enable":1}}\r\n
```

### 2.2. Cấu hình xả pin (cmd_type = 1)

| Tham số        | Kiểu dữ liệu | Mô tả                                |
|----------------|--------------|--------------------------------------|
| current_limit  | uint16       | Dòng điện tối đa                     |
| enable         | uint8        | Cho phép xả pin (0: tắt, 1: bật)     |

Ví dụ:
```json
>{"id":"abc","type":1,"data":{"current_limit":123,"enable":1}}\r\n
```