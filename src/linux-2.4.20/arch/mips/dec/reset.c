/*
 * Reset a DECstation machine.
 */

void (*back_to_prom)(void) = (void (*)(void))0xBFC00000;

void dec_machine_restart(char *command)
{
	back_to_prom();
}

void dec_machine_halt(void)
{
	back_to_prom();
}

void dec_machine_power_off(void)
{
    /* DECstations don't have a software power switch */
	back_to_prom();
}

void dec_intr_halt(int irq, void *dev_id, struct pt_regs *regs)
{
	dec_machine_halt();
}
