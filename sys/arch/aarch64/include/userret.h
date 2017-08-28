/*	$NetBSD$	*/

#include <sys/userret.h>

struct trapframe;

void userret(struct lwp *, struct trapframe *);
