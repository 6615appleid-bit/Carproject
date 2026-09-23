/* �ļ���;��������ڣ���ʼ���󷴸�����CarControl����ǰdebug���������ģ����ԣ����Ǳ���������ڡ� */
#include "Platform.h"
#include "CarControl.h"
/* Power-on auto start. It only retries the same guarded entry point the host
 * uses; CarControl_Start stays rejected until Track and Avoid report usable,
 * stable data, so this never moves the car on its own. Debug commands 2 and 3
 * latch the car off, so an operator stop is not undone by the retry. */
#define AUTO_START_RETRY_MS 500U
/* 1=start, 2=stop, 3=confirmed finish, 4=reset to idle. */
volatile unsigned int debug_command; /* �������1������2ֹͣ��3ȷ���յ㣬4���³�ʼ���� */
volatile unsigned int debug_result; /* ����������������1�ɹ���0δ���������� */
volatile CarState debug_state; /* �ڵ������й۲쵱ǰ����״̬�� */
volatile CarError debug_error; /* �ڵ������й۲����ԭ�� */
/* ������ڣ�Ӧ�ð�ѭ���������������԰�ִ�и��������֤�� */
int main(void)
{
    unsigned int command;
    uint8_t auto_start = 1U; /* Debug commands 2/3 clear it; 1/4 set it again. */
    uint32_t auto_try_ms;
    Platform_Init();
    CarControl_Init();
    auto_try_ms = Platform_GetMs();
    for (;;) {
        command = debug_command;
        debug_command = 0U; /* ����ȡ�������㣬����ͬһ������ѭ�����ظ�ִ�С� */
        switch (command) {
        case 1U: auto_start = 1U; debug_result = CarControl_Start(); break;
        case 2U: auto_start = 0U; CarControl_Stop(); break;
        case 3U: auto_start = 0U; CarControl_ConfirmFinish(); break;
        case 4U: auto_start = 1U; CarControl_Init(); auto_try_ms = Platform_GetMs(); break;
        default: break;
        }
        /* Automatic start is only an attempt: the guarded entry point rejects
         * the request during the 20 s warm-up and until the sensors are
         * usable, so the loop keeps waiting instead of forcing motion. */
        if (auto_start && CarControl_GetState() == CAR_IDLE) {
            uint32_t now = Platform_GetMs();
            if ((uint32_t)(now - auto_try_ms) >= AUTO_START_RETRY_MS) {
                auto_try_ms = now;
                (void)CarControl_Start();
            }
        }
        CarControl_Update();
        debug_state = CarControl_GetState();
        debug_error = CarControl_GetError();
    }
}
