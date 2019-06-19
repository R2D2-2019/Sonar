#pragma once

#include <base_module.hpp>

// This is a test module that sends a request for a distance
// type frame

namespace r2d2::distance {
    class test_module_c : public base_module_c {

    public:
        test_module_c(base_comm_c &comm) : base_module_c(comm) {
        }

        void process() override {
            comm.request(r2d2::frame_type::DISTANCE);
        }
    };
} // namespace r2d2::distance