#include <iostream>

using namespace std;

int main()
{
  for (;;)
  {
  string message;
  getline(cin, message);

  cout << "Program: I've received: "
       << "'" << message  << "'"
       << endl;

  cerr << "Program: This is the error stream!" << endl;
  }
  return (0);
}
