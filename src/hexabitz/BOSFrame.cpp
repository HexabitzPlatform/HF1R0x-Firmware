#include "hexabitz/BOSFrame.h"

hstd::Frame::Frame(void): destID(0), srcID(0), code(0), crc8(0)
{


}

void hstd::Frame::sanitize(void)
{
	if (getTotalLength() == 13)
		param.append(uint8_t(0));

	// TODO: Calculate CRC
	crc8 = CRC8_FIXED; // Fixed Value of CRC
}

void hstd::Frame::setCodeOnly(uint16_t newCode)
{
	const uint16_t clearMask = uint16_t(~ (unsigned(CODE_ONLY_MASK)));
	code &= clearMask;
	code |= (newCode & CODE_ONLY_MASK);
}

uint16_t hstd::Frame::getCodeOnly(void) const
{
	return code & CODE_ONLY_MASK;
}

bool hstd::Frame::isValid(void) const
{
	if (getTotalLength() == 13)
		return false;
	if (crc8 != CRC8_FIXED)
		return false;
	return true;
}

bool hstd::Frame::fromBuffer(BinaryBuffer& buffer)
{
	if (buffer.getLength() < getMinLength())
		return false;
	if (buffer[0] >= buffer.getLength())
		return false;

	uint8_t len = buffer.popui8();

	destID = buffer.popui8();
	srcID = buffer.popui8();
	code = buffer.popui16();
	param.append(buffer, len - getMinLength());
	crc8 = buffer.popui8();

	return isValid();
}

BinaryBuffer hstd::Frame::toBuffer(void) const
{
	BinaryBuffer b;
	BinaryBuffer t(param);

	b.append(uint8_t(getTotalLength()));
	b.append(destID);
	b.append(srcID);
	b.append(code);
	b.append(t);
	b.append(crc8);

	return b;
}

// std::ostream& operator<<(std::ostream& stream, hstd::Frame& f)


std::vector<hstd::Frame> hstd::buildFramesFromMessage(hstd::Message message)
{
	std::vector<hstd::Frame> vec;
	if (!message.getSource().isValid())
		return vec;
	if (!message.getDest().isValid())
		return vec;

	do {
		Frame f;

		int len = hstd::constrain(message.getParams().getLength(), 0, Frame::getMaxParamLength());
		bool isLong = message.getParams().getLength() > Frame::getMaxParamLength();

		f.srcID = static_cast<uint8_t>(message.getSource().getUID());
		f.destID = static_cast<uint8_t>(message.getDest().getUID());

		f.setCodeOnly(message.getCode());

		bitWrite(f.code, Frame::CLI_FLAG_BITPOS, message.getCLIOnlyFlag());
		bitWrite(f.code, Frame::MESS_FLAG_BITPOS, message.getMessOnlyFlag());
		bitWrite(f.code, Frame::TRACE_FLAG_BITPOS, message.getTraceFlag());
		bitWrite(f.code, Frame::LONG_FLAG_BITPOS, isLong);

		f.param.append(message.getParams(), len);
		f.sanitize();

		vec.push_back(f);

	} while (message.getParams().getLength() > 0);

	return vec;
}

bool hstd::buildMessageFromFrames(std::vector<hstd::Frame>& vec, hstd::Message& message)
{
	int i = 0;

	bool isLong = false;
	bool lastFrameLong = false;

	for (i = 0; i < vec.size(); i++) {
		Frame f = vec[i];

		lastFrameLong = f.getFlag(Frame::LONG_FLAG_BITPOS);

		if (!f.isValid())
			return false;
		if (lastFrameLong)
			isLong = true;

		// Process the Frame
		message.setSource(Addr_t(f.srcID));
		message.setDest(Addr_t(f.destID));
		message.setCode(f.getCodeOnly());

		message.setMessOnlyFlag(f.getFlag(Frame::MESS_FLAG_BITPOS));
		message.setCLIOnlyFlag(f.getFlag(Frame::CLI_FLAG_BITPOS));
		message.setTraceFlag(f.getFlag(Frame::TRACE_FLAG_BITPOS));
		message.getParams().append(f.param);

		if (!isLong)
			break;
		if (isLong and !lastFrameLong)
			break;
	}

	if (isLong and lastFrameLong)
		return false;

	vec.erase(vec.begin(), vec.begin() + i);
	return true;
}
