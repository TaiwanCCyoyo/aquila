#ifndef HANDLE_TRAP_H_
#define HANDLE_TRAP_H_
volatile int got_timmer_isr;

void volatile install_isr(unsigned int isr);

//------------------------------
// ISR
void volatile isr();
void volatile isr_is_interrupt();
void volatile isr_is_exception();

//------------------------------
// Interrupt
void volatile isr_is_UserSoftwareInterrupt();
void volatile isr_is_SupervisorSoftwareInterrupt();
void volatile isr_is_MachineSoftwareInterrupt();
void volatile isr_is_UserTimerInterrupt();
void volatile isr_is_SupervisorTimerInterrupt();
void volatile isr_is_MachineTimerInterrupt();
void volatile isr_is_UserExternalInterrupt();
void volatile isr_is_SupaervisorExternalInterrupt();
void volatile isr_is_MachineExternalInterrupt();
void volatile isr_is_UnknownInterrupt();

//------------------------------
// Exception
void volatile isr_is_InstructionAddressMisaligned();
void volatile isr_is_InstructionAccessFault();
void volatile isr_is_IllegalInstruction();
void volatile isr_is_Breakpoint();
void volatile isr_is_LoadAddressMisaligned();
void volatile isr_is_LoadAccessFault();
void volatile isr_is_StoreAmoAddressMisaligned();
void volatile isr_is_StoreAmoAccessFault();
void volatile isr_is_EnvironmentCallFromUmode();
void volatile isr_is_EnvironmentCallFromSmode();
void volatile isr_is_EnvironmentCallFromMmode();
void volatile isr_is_InstructionPageFault();
void volatile isr_is_LoadPageFault();
void volatile isr_is_StoreAmoPageFault();
void volatile isr_is_UnknownException();

//----------------------------------
// My return
void my_mret();
#endif