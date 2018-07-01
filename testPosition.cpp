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

    cout << "Test de positionnement d'un servomoteur \n";
    while (!ok)
    {
        cout << "Entrer l'id su servo à positionner : ";
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
    
    servo.setGoalVelocity(1023);
    valeur = servo.readPosition();

    cout << "\nPosition actuelle : " << valeur << endl;

    while (ok)
    {
        cout << "Entrer une nouvelle position : ";
        cin >> valeur;
        if ((valeur >= 0) && (valeur < 1024))
        {
            servo.setGoalPosition(valeur);
            servo.setTorque(XL320_TORQUE::ON);
            servo.setLed(XL320_LED::RED);
            while (servo.isMoving()) delay(10);
            servo.setLed(XL320_LED::GREEN);
        }
        else
            ok = false;
    }
    servo.setTorque(XL320_TORQUE::OFF);
    servo.setLed(XL320_LED::NONE);

    cout << "Bye" << endl;
}
