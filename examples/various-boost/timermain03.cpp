/**
 * @file timermain03.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "timer03.h"

int main(int argc, char* argv[])
{
	set_priority(0x01);

	boost::asio::io_context ioContext;

	print(PRI1, "main: Timer02 c1(ioContext);\n");
	Timer03 c1(ioContext);

	print(PRI1, "main: c1.start();\n");
	c1.start();

	print(PRI1, "main: before ioContext.run();\n");
	ioContext.run();
	print(PRI1, "main: after ioContext.run();\n");

	print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
	std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
	return 0;
}
