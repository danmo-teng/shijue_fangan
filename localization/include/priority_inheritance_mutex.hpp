#pragma once

#include <pthread.h>
#include <system_error>

namespace omni {

// Shared by FIFO owners and ordinary callers. Unsupported kernels retain the
// ordinary mutex and expose that result; no sensor/control admission policy.
class PriorityInheritanceMutex {
public:
    PriorityInheritanceMutex() {
        pthread_mutexattr_t attributes;
        int error=pthread_mutexattr_init(&attributes);
        if(!error){
            error=pthread_mutexattr_setprotocol(&attributes,PTHREAD_PRIO_INHERIT);
            if(!error){
                error=pthread_mutex_init(&value_,&attributes);
                priority_inheritance_=error==0;
            }
            pthread_mutexattr_destroy(&attributes);
        }
        protocol_error_=error;
        if(!priority_inheritance_){
            error=pthread_mutex_init(&value_,nullptr);
            if(error)throw std::system_error(error,std::generic_category(),"native motion mutex initialization");
        }
    }
    ~PriorityInheritanceMutex(){pthread_mutex_destroy(&value_);}
    PriorityInheritanceMutex(const PriorityInheritanceMutex&)=delete;
    PriorityInheritanceMutex &operator=(const PriorityInheritanceMutex&)=delete;
    void lock(){const int error=pthread_mutex_lock(&value_);if(error)throw std::system_error(error,std::generic_category(),"native motion mutex lock");}
    void unlock()noexcept{pthread_mutex_unlock(&value_);}
    bool priority_inheritance()const noexcept{return priority_inheritance_;}
    int protocol_error()const noexcept{return protocol_error_;}
private:
    pthread_mutex_t value_{};bool priority_inheritance_=false;int protocol_error_=0;
};

} // namespace omni
