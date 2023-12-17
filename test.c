int sum(int x, int y) {
	return x + y;
}

// int sub(int x, int y) {
// 	return x - y;
// }

// int neg(int x) {
// 	return -x;
// }

// int or(int x, int y) {
// 	return x | y;
// }

// int not(int x) {
// 	return ~x;
// }

// int and(int x, int y) {
// 	return x & y;
// }


int main() {
	int a;
	int b, c, d;
	a = 32;
	b = 399;
	c = sum(a, b);
	return c;
}

// .org 4096
// STACK: .dw 1024
// .org 0
// la r1, STACK
// la r2, func1
// br r2
// func0: ;sum
// ld r3, 0(r1)
// addi r1, r1, 4
// ld r4, 0(r1)
// addi r1, r1, 4
// add r3, r3, r4
// addi r1, r1, -4
// st r3, 0(r1)
// br r2
// func1: ;main
// addi r7, r0, 99
// addi r1, r1, -4
// st r7, 0(r1)
// ld r5, 0(r1)
// addi r1, r1, 4
// addi r7, r0, 88
// addi r1, r1, -4
// st r7, 0(r1)
// ld r6, 0(r1)
// addi r1, r1, 4
// addi r1, r1, -4
// st r5, 0(r1)
// addi r1, r1, -4
// st r6, 0(r1)
// la r7, func0
// brl r2, r7
// ld r7, 0(r1)
// addi r1, r1, 4
// addi r1, r1, -4
// st r7, 0(r1)
// br r2 ; Should be removed, calling convention wip
// stop