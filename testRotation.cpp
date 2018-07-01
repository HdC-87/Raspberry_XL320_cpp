#include "XL320.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iomanip>

using namespace std;

int valeur = 0;
char cr = 13;
bool ok = false;
int main()
{
    int fd;

    if ((fd = serialOpen("/dev/ttyAMA0", 1000000)) < 0)
    {
        cout << "Erreur ouverture port série " << endl;
        return 1;
    }

    if (wiringPiSetup() == -1)
    {
        cout << " Erreur ouverture wiringPi" << endl;
        return 1;
    }

    cout << "Test de rotation d'un servomoteur \n";
    cout << "Vitesse de rotation v de 0 à 1023 : n = 0,111 x v sens CCW" << endl;
    cout << "Vitesse de rotation v de 1024 à 2047 : n = 0,111 x (v-1024) sens CW" << endl;

    while (!ok)
    {
        cout << "Entrer l'id su servo à tester : ";
        cin >> valeur;
        XL320 servo(fd, valeur);
        if (servo.setLed(XL320_LED::NONE))
            ok = true;
        else
        {
            cout << "Le servo n'esiste pas ... \n" << endl;
        }
    }

    XL320 servo(fd, valeur);

    servo.setTorque(XL320_TORQUE::OFF);
    servo.setMode(XL320_CONTROL_MODE::WHEEL);
    servo.setTorque(XL320_TORQUE::ON);
    servo.setGoalVelocity(0);

    while (ok)
    {
        cout << "Entrer une nouvelle consigne de vitesse de rotation : ";
        cin >> valeur;
        if ((valeur >= 0) && (valeur < 2048))
        {
            servo.setGoalVelocity(valeur);
        }
        else
            ok = false;
    }

    servo.setGoalVelocity(0);
    cout << "Bye" << endl;
}
