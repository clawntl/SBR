// Run a function in a short-lived task pinned to one core and wait for it.
//
// Spec §8.2 rule 1: interrupts are allocated on the core that installs them,
// so SPI, the GPIO ISR service and can0.begin() are set up from core 1.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace RunOnCore {

namespace detail {
struct Job {
    void (*fn)(void *);
    void *arg;
    TaskHandle_t waiter;
};

inline void trampoline(void *p)
{
    Job *job = static_cast<Job *>(p);
    job->fn(job->arg);
    xTaskNotifyGive(job->waiter);
    vTaskDelete(nullptr);
}
}  // namespace detail

// Blocks the caller until fn(arg) has returned on `core`. Uses the caller's
// task notification. Returns false if the task couldn't be created (fn not run).
inline bool run(BaseType_t core, void (*fn)(void *), void *arg, const char *name = "oncore",
                uint32_t stackBytes = 4096)
{
    detail::Job job{fn, arg, xTaskGetCurrentTaskHandle()};
    if (xTaskCreatePinnedToCore(detail::trampoline, name, stackBytes, &job,
                                uxTaskPriorityGet(nullptr), nullptr, core) != pdPASS) {
        return false;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return true;
}

}  // namespace RunOnCore
