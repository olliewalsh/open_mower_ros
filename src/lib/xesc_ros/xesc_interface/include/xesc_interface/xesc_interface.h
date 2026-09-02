//
// Created by clemens on 02.07.22.
//

#ifndef SRC_XESC_INTERFACE_H
#define SRC_XESC_INTERFACE_H

#include <functional>

#include <xesc_msgs/XescStateStamped.h>


namespace xesc_interface {
    class XescInterface {
    public:
        using StatusCallback = std::function<void(const xesc_msgs::XescStateStamped &)>;

        virtual void getStatus(xesc_msgs::XescStateStamped &state)=0;
        virtual void getStatusBlocking(xesc_msgs::XescStateStamped &state)=0;
        virtual void setDutyCycle(float duty_cycle)=0;
        virtual void stop()=0;

        virtual bool supportsStatusRequests() const {
            return false;
        }

        virtual void setStatusCallback(const StatusCallback &callback) {
            (void)callback;
        }

        virtual void requestStatus() {
        }

    };
}

#endif
