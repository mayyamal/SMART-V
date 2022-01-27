

void f1();
void f2();

int _start() {
	asm volatile(" li sp, 5242880" ::: "sp");
	f1();
}
	
void f1() {
	int a = 1;
	int b = 2;
	f2();
	int sum = a + b;
	
}

void f2() {
	int a = 1;
	int b = 2;
	int sum = a + b;
}


