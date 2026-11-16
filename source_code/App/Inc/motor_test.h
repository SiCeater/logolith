#ifndef __MOTOR_TEST_H__
#define __MOTOR_TEST_H__

/* Test de correspondance moteurs (HÉLICES DÉMONTÉES), voir motor_test.c.
 * Drone armé + gaz en bas + appui SD → M1..M4 l'un après l'autre à 20 %.
 * À appeler dans la slow loop, après Security_Process(). Non bloquant. */
void MotorTest_Process(void);

#endif
