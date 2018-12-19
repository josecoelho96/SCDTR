//
//  hello world.cpp
//  scdtr
//
//  Created by David Teles on 08/11/2018.
//

#include <iostream>
#include <pigpio.h>
using namespace std;
int main (int argc, char *argv[])
{
    if ( gpioInitialise() < 0 ){
        cout << "Error Initialising pigpio" << endl;
		return 1; //failed
	}

auto hw = gpioHardwareRevision();
auto sw = gpioVersion();
cout << "Hello from pigpio V" << sw << "on a PI rev. " << hw << endl;
gpioTerminate();
}
