from smbus2 import SMBus, i2c_msg
import struct

# Opcodes mapping (must match Arduino Wire implementation)
OP_HOME = 0x01
OP_QUERY = 0x02
OP_STEP_UP = 0x03
OP_STEP_DOWN = 0x04
OP_MOVE_STEPS = 0x05
OP_SET_POS = 0x06
OP_SET_OBJECTIVE = 0x07
OP_SET_RPM = 0x08
OP_EEPROM_WRITE = 0x09

class I2CController:
    def __init__(self, addr: int, bus_id: int = 1):
        self.addr = addr
        self.bus_id = bus_id

    def _write_frame(self, opcode: int, payload: bytes = b""):
        frame = bytes([opcode, len(payload)]) + payload
        # write_i2c_block_data: first byte is 'command', rest is data
        with SMBus(self.bus_id) as bus:
            bus.write_i2c_block_data(self.addr, frame[0], list(frame[1:]))

    def _read_resp(self, max_len: int = 32):
        with SMBus(self.bus_id) as bus:
            msg = i2c_msg.read(self.addr, max_len)
            bus.i2c_rdwr(msg)
            data = bytes(list(msg))
        if not data:
            return 1, b""
        status = data[0]
        ln = data[1] if len(data) > 1 else 0
        return status, data[2:2+ln]

    # High-level helpers mirroring serial API
    def home(self, raise_steps: int = 0):
        self._write_frame(OP_HOME, struct.pack('<h', int(raise_steps)))

    def get_position(self) -> int:
        self._write_frame(OP_QUERY, b"")
        st, payload = self._read_resp(16)
        if st != 0 or len(payload) < 6:
            return 0
        # Example payload layout: [objective:u8][homed:u8][pos:i32]
        pos = struct.unpack_from('<i', payload, 2)[0]
        return pos

    def get_max_limit(self) -> int:
        # Reuse QUERY payload if it contains limit; otherwise return a safe default
        self._write_frame(OP_QUERY, b"")
        st, payload = self._read_resp(24)
        if st != 0 or len(payload) < 10:
            return 20000
        # Example extended: [objective:u8][homed:u8][pos:i32][limit:i32]
        limit = struct.unpack_from('<i', payload, 6)[0]
        return limit

    def move_steps(self, steps: int):
        self._write_frame(OP_MOVE_STEPS, struct.pack('<i', int(steps)))

    def set_objective(self, obj: str):
        # map 4/10/40 -> 0/1/2 for compact payload (adjust to your firmware)
        m = {'4':0,'10':1,'40':2}
        code = m.get(str(obj), 2)
        self._write_frame(OP_SET_OBJECTIVE, bytes([code]))

    def set_rpm(self, rpm: int):
        self._write_frame(OP_SET_RPM, bytes([max(0, min(255, int(rpm)))]))

    # Convenience for console ASCII bridge if firmware supports it (optional)
    def send_ascii(self, cmd: str) -> str:
        # If your firmware supports an ASCII passthrough, define an opcode and implement it there.
        # Here we emulate common commands via structured ops for known ones.
        c = cmd.strip().upper()
        if c == 'Z':
            self.home(0); return 'Homing...'
        if c.startswith('G'):
            try:
                self.move_steps(int(c[1:]))
                return f"Moved {int(c[1:])}"
            except: return 'ERR'
        if c == 'Q':
            st, payload = self._read_resp(24)
            return payload.decode('ascii', errors='ignore') if payload else ''
        if c.startswith('O'):
            self.set_objective(c[1:]); return f"Objective {c[1:]}"
        if c.startswith('V'):
            self.set_rpm(int(c[1:])); return f"RPM set"
        return ''
