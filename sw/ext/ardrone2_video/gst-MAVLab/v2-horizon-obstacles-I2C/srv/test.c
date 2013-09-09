autorun(5);
printf("test 1\n");
printf("hello\n");
printf("hello\n"); /* this is a comment */ printf("hello\n");
printf("hello\n");
// this is also a comment printf("hello\n");
printf("hello\n");

printf("test 2\n");
int Count;
for (Count = -5; Count <= 5; Count++)
    printf("Count = %d\n", Count);

printf("String 'hello', 'there' is '%s', '%s'\n", "hello", "there");
printf("Character 'A' is '%c'\n", 65);
printf("Character 'a' is '%c'\n", 'a');

printf("test 3\n");
struct fred
{
    int boris;
    int natasha;
};

struct fred bloggs;

bloggs.boris = 12;
bloggs.natasha = 34;

printf("bloggs.boris = %d\n", bloggs.boris);
printf("bloggs.natasha = %d\n", bloggs.natasha);

printf("test 5\n");
int Array[10];

for (Count = 1; Count <= 10; Count++)
{
    Array[Count-1] = Count * Count;
}

for (Count = 0; Count < 10; Count++)
{
    printf("%d\n", Array[Count]);
}

printf("test 6\n");
for (Count = 0; Count < 4; Count++)
{
    printf("%d\n", Count);
    switch (Count)
    {
        case 1:
            printf("1\n");
	    break;

        case 2:
            printf("2\n");
	    break;

        default:
            printf("0\n");
            break;
    }
}

printf("test 7\n");
int myfunc(int x)
{
    return x * x;
}

printf("3x3 = %d\n", myfunc(3));
printf("4x4 = %d\n", myfunc(4));

printf("test 8\n");
int o;
int p;
int q;

o = 1;
p = 0;
q = 0;

while (o < 100)
{
    printf("%d\n", o);
    q = o;
    o = q + p;
    p = q;
}

printf("test 9\n");
int x;
int s;
int t;

x = 1;
s = 0;
t = 0;

do
{
    printf("%d\n", x);
    t = x;
    x = t + s;
    s = t;
} while (x < 100);

printf("test 10\n");
int a;
int *b;
int c;

a = 42;
b = &a;
printf("a = %d\n", *b);

struct ziggy
{
    int a;
    int b;
    int c;
} bolshevic;

bolshevic.a = 12;
bolshevic.b = 34;
bolshevic.c = 56;

printf("bolshevic.a = %d\n", bolshevic.a);
printf("bolshevic.b = %d\n", bolshevic.b);
printf("bolshevic.c = %d\n", bolshevic.c);

/*
b = &(bolshevic.b);
printf("bolshevic.b = %d\n", *b);
*/

printf("test11\n");
laser(1);
motors(50, -50);
delay(500);
motors(0, 0);
laser(0);
printf("time = %d\n", time());
printf("rand(10) = %d\n", rand(10));

exit();

