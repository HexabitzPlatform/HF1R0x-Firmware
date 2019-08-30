#include "hexabitz/BOSMessage.h"

#include "helper/helper.h"



hstd::Message::Message(void): src_(), dest_(), code_(0)
{

}

hstd::Message::Message(Addr_t dest, Addr_t src, uint16_t code): src_(src), dest_(dest), code_(code)
{

}

hstd::Message::Message(uint8_t dest, uint8_t src, uint16_t code): src_(src), dest_(dest), code_(code)
{

}


void hstd::Message::setFlag(std::string name, bool value)
{
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); } );

	flags_[name] = value;
}

bool hstd::Message::getFlag(std::string name)
{
	std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); } );

	if (!flags_.count(name))
		setFlag(name, false);

	return flags_[name];
}

void hstd::Message::invalidate(void)
{
	src_ = Addr_t();
	dest_ = Addr_t();
	code_ = 0;

	param_.reset();
	flags_.clear();
}


std::string hstd::Message::to_string(void) const
{
	std::stringstream stream;
	stream << "<<<< Src (ID: " << src_.getUID() << ", Port: " << src_.getPort() << ") | ";
	stream << "Dest (ID: " << dest_.getUID() << ", Port: " << dest_.getPort() << ") | ";
	stream << "Code (0x"  << std::hex << int(code_) << ") | ";

	stream << "Flags (";
	for (auto& [name, value]: flags_)
		stream << " (" << name << ", " << (value ? "true" : "false") << ")";
	stream << " ) | ";

	stream << "Parameters ( ";
	for (int i = 0; (i < param_.getLength()); i++)
		stream << std::hex << "0x" << int(param_[i]) << " ";

	stream << ") >>>>";

	return stream.str();
}


// std::vector<hstd::Message> hstd::getSanitizedMessages(hstd::Message msg)
// {
// 	std::vector<hstd::Message> list;
// 	const size_t maxParamLen = 50 - Frame::getMinLength();

// 	do {
// 		int len = hstd::constrain(msg.getParams().getLength(), 0, maxParamLen);
// 		bool isLong = msg.getParams().getLength() > maxParamLen;

// 		hstd::Message smallPkt;
// 		smallPkt.setSource(msg.getSource());
// 		smallPkt.setDest(msg.getDest());
// 		smallPkt.setCode(msg.getCode());
// 		smallPkt.setLongFlag(isLong);
// 		smallPkt.getParams().append(msg.getParams(), len);
// 		smallPkt.sanitize();

// 		list.push_back(smallPkt);
// 	} while (msg.getParams().getLength());

// 	return list;
// }

// bool hstd::parseMessage(BinaryBuffer buffer, hstd::Message& msg)
// {
// 	std::vector<hstd::Message> list;

// 	while (buffer.getLength()) {
// 		hstd::Message temp;
// 		if (!temp.parse(buffer))
// 			break;
// 		// if (!temp.validate())
// 		// 	return false;

// 		list.push_back(temp);
// 		if (!temp.getLongFlag())
// 			break;
// 	}
// 	if (list.size() <= 0)
// 		return false;
// 	if (list.back().getLongFlag())
// 		return false;

// 	// TODO: Test all have same src, dest and code
// 	msg.reset();

// 	msg.setSource(list.front().getSource());
// 	msg.setDest(list.front().getDest());
// 	msg.setCode(list.front().getCode());
// 	msg.setLongFlag(false);

// 	for (auto& i: list)
// 		msg.getParams().append(i.getParams());

// 	return true;
// }

// hstd::Message hstd::buildMessage(uint8_t dst, uint8_t src, BinaryBuffer buffer)
// {
// 	hstd::Message msg;
// 	msg.setSource(src);
// 	msg.setDest(dst);
// 	msg.getParams().append(buffer);
// 	msg.sanitize();
// 	return msg;
// }