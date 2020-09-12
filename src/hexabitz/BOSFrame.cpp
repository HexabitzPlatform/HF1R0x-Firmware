#include "hexabitz/BOSFrame.h"
#include <cstring>
hstd::Frame::Frame(void): destID(0), srcID(0), code(0), crc8(0)
{


}

void hstd::Frame::sanitize(void)
{
	if (getTotalLength() == 13)
		param.append(uint8_t(0));

	// TODO: Calculate CRC from Buffer
	crc8 = CRC8_FIXED; // Fixed Value of CRC
}

uint32_t crc32b(uint8_t *message, size_t l,unsigned int crc)
{
   size_t i, j;
   unsigned int msb;

   for(i = 0; i < l; i++) {
      // xor next byte to upper bits of crc
      crc ^= (((unsigned int)message[i])<<24);
      for (j = 0; j < 8; j++) {    // Do eight times.
            msb = crc>>31;
            crc <<= 1;
            crc ^= (0 - msb) & 0x4C11DB7;
      }
   }
   return crc;         // don't complement crc on output
}
uint8_t *reverse_array( uint8_t *array , uint8_t length){
	static uint8_t array_temp[8];
	array[length-1]=0x00;
	uint8_t jj=3;
	for(int ii=0;ii<4;ii++){
			array_temp[ii]=array[jj];
		jj--;
		}
		
	for(int kk=4;kk<8;kk++){
			array_temp[kk]=array[length-1];
		length--;
		}
		return array_temp;
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
	
	// uint8_t codeMSB = buffer.popui8();
	// uint8_t codeLSB = buffer.popui8();
	// code = (uint16_t(codeMSB) << 8) | codeLSB;

	param.append(buffer, len - getMinLength());
	crc8 = buffer.popui8();

	return isValid();
}

BinaryBuffer hstd::Frame::toBuffer(void) const
{
	BinaryBuffer b;
	BinaryBuffer t(param);
	
	b.append(uint8_t('H'));
	b.append(uint8_t('Z'));

	b.append(uint8_t(getTotalLength()));
	b.append(destID);
	b.append(srcID);

	// b.append(uint8_t(highByte(code)));
	// b.append(uint8_t(lowByte(code)));
	// Edit Default Options  
	uint8_t option = 0;
	b.append(option);
	b.append(uint8_t(1));
	//b.append(uint8_t(0));
	BinaryBuffer bb;
	int bL=b.getLength();
	uint8_t buf[bL+1];
	uint8_t *buf1;
	uint8_t *buf2;
	std::cout<<"getTotalLength="<<bL<<std::endl;
	for(int k=0;k<=6;k++){
		uint8_t db = b.popui8();
		buf[k] = db;
		bb.append(db);
		}
	// TODO: Add CRC32 byte to Buffer
	buf1=reverse_array(buf,8);
	uint32_t crc = crc32b(buf1, 8,0xFFFFFFFF);
	std::cout<<"CRC1: "<<"0x"<<crc<<std::endl;
	bb.append(uint8_t(crc));
	return bb;
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

		// TODO: Change Options
		//bitWrite(f.code, Frame::CLI_FLAG_BITPOS, message.getCLIOnlyFlag());
		//bitWrite(f.code, Frame::MESS_FLAG_BITPOS, message.getMessOnlyFlag());
		//bitWrite(f.code, Frame::TRACE_FLAG_BITPOS, message.getTraceFlag());
		//bitWrite(f.code, Frame::LONG_FLAG_BITPOS, isLong);

		//f.param.append(message.getParams(), len);
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

	message.invalidate();

	for (i = 0; i < vec.size(); i++) {
		Frame f = vec[i];

		lastFrameLong = f.getFlag(Frame::LONG_FLAG_BITPOS);

		if (!f.isValid())
			return false;
		if (lastFrameLong)
			isLong = true;

		// Process the Frame
		// TODO: Check Source and Dest of Long Frames, Code. They should be same.
		message.setSource(Addr_t(f.srcID));
		message.setDest(Addr_t(f.destID));
		message.setCode(f.getCodeOnly());


		// TODO: Change Options
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
