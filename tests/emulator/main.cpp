// Board tests with hand-encoded SH-1 programs. No firmware ROM is needed.
#include "test.h"

int failures = 0, assertions = 0;

int main()
{
    for (const auto *tests : {&cpu_tests, &interrupt_tests, &exception_tests, &sh7034_tests, &serial_tests,
                              &board_tests, &timing_tests, &drive_tests, &api_tests})
    {
        for (const auto &t : *tests)
        {
            const int before = failures;
            t.run();
            std::printf("%s %s\n", failures == before ? "ok  " : "FAIL", t.name);
        }
    }
    std::printf("%d assertions, %d failed\n", assertions, failures);
    return failures ? 1 : 0;
}
