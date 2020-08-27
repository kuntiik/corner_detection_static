#include<iostream>
#include<chrono>
#include<thread>

#define LOG(x) std::cout << x << std::endl;

static bool s_terminate = false;
static bool s_screen = false;

void spam(){
    using namespace std::literals::chrono_literals;
    while(!s_terminate){
        if(s_screen){
            LOG("pressed a  now do foo");
            s_screen = false;
        }
        else{
        std::cout << "doint foo" << std::endl;
        }
        std::this_thread::sleep_for(1s);
    }
}

void keyboard(){
    char c;
    while(!s_terminate){
        c = std::cin.get();
        if(c == 'a'){ 
            s_screen = true;
        }
        else if (c == 'q') {
            s_terminate = true;
        }
    }
}

    
int main(int argc, char**argv){

    std::thread worker(spam);
    std::thread keyb(keyboard);

    worker.join();
    keyb.join();
    std::cout << "terminated both threads" << std::endl;


    std::cin.get();
}
