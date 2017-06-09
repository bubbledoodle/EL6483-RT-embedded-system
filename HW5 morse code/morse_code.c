#include <stdio.h>
typedef struct _MorseInfo
{
  char letter;
  int code;
}morseInfo;


morseInfo aplhabet[26];
char charTable[26] = {'A','B','C','D',
                      'E','F','G','H',
                      'I','G','K','L',
                      'M','N','O','P',
                      'Q','R','S','T',
                      'U','V','W','X',
                      'Y','Z'};

int morseTable[26] = {1200,2111,2121,2110, // ABCD
                      1000,1121,2210,1111, // EFGH
                      1100,1222,2120,1211, // IJKL
                      2200,2100,2220,1221, // MNOP
                      2212,1210,1110,2000, // QRST
                      1120,1112,1220,2112, // UVWX
                      2122,2211};      // XYZ

void SetMorseTable(morseInfo aplhabet[])
{
  for(int i = 0; i < 26; i++)
  {
  aplhabet[i].letter = charTable[i];
  aplhabet[i].code = morseTable[i];
  }

}
