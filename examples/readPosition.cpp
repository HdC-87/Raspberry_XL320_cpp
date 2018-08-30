#include "XL320.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iomanip>

using namespace std;

int valeur = 0;
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

    cout << "Lecture de la position d'un servomoteur \n";
    while (!ok)
    {
        cout << "Entrer l'id su servo : ";
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
    servo.setMode(XL320_CONTROL_MODE::JOIN);
    

    while (ok)
    {
        valeur = servo.readPosition();
        cout << "\nPosition actuelle : " << valeur << endl;
        delay(250);
    }

    cout << "Bye" << endl;
}
