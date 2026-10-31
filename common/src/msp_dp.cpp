#include "mabur/msp_dp.h"

#include <algorithm>

namespace mabur {

void msp_append_message(std::vector<uint8_t>& out, uint8_t cmd,
                        const uint8_t* payload, size_t n) {
  out.push_back('$');
  out.push_back('M');
  out.push_back('<');
  out.push_back(static_cast<uint8_t>(n));
  out.push_back(cmd);
  uint8_t cks = static_cast<uint8_t>(n) ^ cmd;
  for (size_t i = 0; i < n; ++i) {
    out.push_back(payload[i]);
    cks ^= payload[i];
  }
  out.push_back(cks);
}

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
  return crc;
}

void msp_append_message_v2(std::vector<uint8_t>& out, uint16_t cmd,
                           const uint8_t* payload, size_t n) {
  out.push_back('$');
  out.push_back('X');
  out.push_back('<');
  const uint8_t hdr[5] = {0, static_cast<uint8_t>(cmd & 0xFF), static_cast<uint8_t>(cmd >> 8),
                          static_cast<uint8_t>(n & 0xFF), static_cast<uint8_t>(n >> 8)};
  uint8_t crc = 0;
  for (uint8_t h : hdr) {
    out.push_back(h);
    crc = msp_crc8_dvb_s2(crc, h);
  }
  for (size_t i = 0; i < n; ++i) {
    out.push_back(payload[i]);
    crc = msp_crc8_dvb_s2(crc, payload[i]);
  }
  out.push_back(crc);
}

std::vector<MspMessage> MspParser::feed(const uint8_t* p, size_t n) {
  std::vector<MspMessage> out;
  for (size_t i = 0; i < n; ++i) {
    uint8_t dat = p[i];
    switch (state_) {
      default:
      case IDLE:
        if (dat == '$') state_ = VERSION;
        break;
      case VERSION:
        state_ = (dat == 'M') ? DIRECTION : (dat == 'X') ? V2_DIRECTION : IDLE;
        break;
      case V2_DIRECTION:
        // '<' request / '>' reply / '!' error reply; anything else is garbage.
        state_ = (dat == '<' || dat == '>' || dat == '!') ? V2_FLAG : IDLE;
        break;
      case V2_FLAG:
        checksum_ = msp_crc8_dvb_s2(0, dat);
        state_ = V2_CMD_LO;
        break;
      case V2_CMD_LO:
        cmd16_ = dat;
        checksum_ = msp_crc8_dvb_s2(checksum_, dat);
        state_ = V2_CMD_HI;
        break;
      case V2_CMD_HI:
        cmd16_ |= static_cast<uint16_t>(dat) << 8;
        checksum_ = msp_crc8_dvb_s2(checksum_, dat);
        state_ = V2_SIZE_LO;
        break;
      case V2_SIZE_LO:
        size16_ = dat;
        checksum_ = msp_crc8_dvb_s2(checksum_, dat);
        state_ = V2_SIZE_HI;
        break;
      case V2_SIZE_HI:
        size16_ |= static_cast<uint16_t>(dat) << 8;
        // Both firmwares cap a frame at MSP_PORT_OUTBUF_SIZE (512): one
        // corrupted size byte must not park the parser in V2_PAYLOAD for up
        // to 64 KB, i.e. seconds of swallowed DisplayPort and replies.
        if (size16_ > 512) {
          state_ = IDLE;
          break;
        }
        checksum_ = msp_crc8_dvb_s2(checksum_, dat);
        ptr16_ = 0;
        payload_.assign(size16_, 0);
        state_ = size16_ > 0 ? V2_PAYLOAD : V2_CRC;
        break;
      case V2_PAYLOAD:
        payload_[ptr16_] = dat;
        checksum_ = msp_crc8_dvb_s2(checksum_, dat);
        if (++ptr16_ == size16_) state_ = V2_CRC;
        break;
      case V2_CRC:
        if (checksum_ == dat) {
          MspMessage m;
          m.cmd = 0;
          m.payload = payload_;
          m.v2 = true;
          m.cmd16 = cmd16_;
          out.push_back(std::move(m));
        }
        state_ = IDLE;
        break;
      case DIRECTION:
        // '<' command / '>' reply; anything else is garbage.
        if (dat == '<' || dat == '>') state_ = SIZE;
        else state_ = IDLE;
        break;
      case SIZE:
        size_ = dat;
        checksum_ = dat;
        state_ = CMD;
        break;
      case CMD:
        cmd_ = dat;
        checksum_ ^= dat;
        buf_ptr_ = 0;
        payload_.assign(size_, 0);
        state_ = (size_ > 0) ? PAYLOAD : CHECKSUM;
        break;
      case PAYLOAD:
        payload_[buf_ptr_] = dat;
        checksum_ ^= dat;
        if (++buf_ptr_ == size_) state_ = CHECKSUM;
        break;
      case CHECKSUM:
        if (checksum_ == dat) {
          MspMessage m;
          m.cmd = cmd_;
          m.payload = payload_;
          m.cmd16 = cmd_;
          out.push_back(std::move(m));
        }
        state_ = IDLE;
        break;
    }
  }
  return out;
}

void MspScreen::set_canvas(uint8_t hd_option) {
  // msp_displayport.h msp_hd_options_e: 0=SD 30x16, 1=HD 50x18, 2=HD 30x16,
  // 3=HD 60x22.
  int cols = 50, rows = 18;
  switch (hd_option) {
    case 0: cols = 30; rows = 16; break;
    case 1: cols = 50; rows = 18; break;
    case 2: cols = 30; rows = 16; break;
    case 3: cols = 60; rows = 22; break;
    default: cols = 50; rows = 18; break;
  }
  if (cols != cols_ || rows != rows_) {
    cols_ = cols;
    rows_ = rows;
    buf_.assign(static_cast<size_t>(rows_) * static_cast<size_t>(cols_), 0);
  }
  hd_option_ = hd_option;
}

bool MspScreen::apply(const MspMessage& m) {
  if (m.cmd != MSP_CMD_DISPLAYPORT || m.payload.empty()) return false;
  const auto& p = m.payload;
  switch (p[0]) {
    case MSP_DP_CLEAR:
      std::fill(buf_.begin(), buf_.end(), static_cast<uint16_t>(0));
      return false;
    case MSP_DP_DRAW_STRING: {
      if (p.size() < 4) return false;
      int row = p[1], col = p[2];
      uint8_t attrs = p[3];
      uint16_t page = static_cast<uint16_t>((attrs & 0x3) << 8);
      for (size_t i = 4; i < p.size(); ++i) {
        int c = col + static_cast<int>(i - 4);
        if (row >= 0 && row < rows_ && c >= 0 && c < cols_)
          buf_[static_cast<size_t>(row) * cols_ + c] =
              static_cast<uint16_t>(p[i]) | page;
      }
      return false;
    }
    case MSP_DP_SET_OPTIONS:
      if (p.size() >= 3) { font_ = p[1]; set_canvas(p[2]); }
      return false;
    case MSP_DP_DRAW_SCREEN:
      return true;
    default:
      return false;  // KEEPALIVE / CLOSE / unknown: no buffer change
  }
}

std::vector<uint8_t> MspScreen::serialize_snapshot(size_t max_bytes,
                                                   bool* truncated) const {
  std::vector<uint8_t> out;
  bool trunc = false;
  auto fits = [&](size_t extra) { return out.size() + extra <= max_bytes; };

  // CLEAR and SET_OPTIONS are the mandatory header; like every other frame
  // below, they only go out if they fit the budget (deviation from the
  // brief's original transcription, which emitted them unconditionally and
  // could blow max_bytes on its own before any per-row check ran).
  if (fits(1 + 6)) {
    uint8_t clr = MSP_DP_CLEAR;
    msp_append_message(out, MSP_CMD_DISPLAYPORT, &clr, 1);
  } else {
    trunc = true;
  }
  if (fits(3 + 6)) {
    uint8_t opt[3] = {MSP_DP_SET_OPTIONS, font_, hd_option_};
    msp_append_message(out, MSP_CMD_DISPLAYPORT, opt, 3);
  } else {
    trunc = true;
  }

  for (int r = 0; r < rows_ && !trunc; ++r) {
    int c = 0;
    while (c < cols_) {
      uint16_t v = cell(r, c);
      if (is_blank(v)) { ++c; continue; }
      uint8_t page = static_cast<uint8_t>((v >> 8) & 0x3);
      // Run of non-blank cells sharing the same page.
      std::vector<uint8_t> payload = {MSP_DP_DRAW_STRING,
                                      static_cast<uint8_t>(r),
                                      static_cast<uint8_t>(c), page};
      int start = c;
      while (c < cols_ && !is_blank(cell(r, c)) &&
             static_cast<uint8_t>((cell(r, c) >> 8) & 0x3) == page) {
        payload.push_back(static_cast<uint8_t>(cell(r, c) & 0xFF));
        ++c;
      }
      // MSP frame overhead is 6 bytes ($M< size cmd cks) around the payload.
      if (!fits(payload.size() + 6)) { trunc = true; c = start; break; }
      msp_append_message(out, MSP_CMD_DISPLAYPORT, payload.data(), payload.size());
    }
  }

  { uint8_t ds = MSP_DP_DRAW_SCREEN;
    if (fits(1 + 6)) msp_append_message(out, MSP_CMD_DISPLAYPORT, &ds, 1);
    else trunc = true; }

  if (truncated) *truncated = trunc;
  return out;
}

}  // namespace mabur
