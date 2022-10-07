# BLE - ITS

Este repositorio contiene dos proyectos en los que se integra la tecnología Bluetooth Low Energy en los Sistemas Inteligentes de Transporte. El marco de pruebas permite validar BLE para diferentes casos de uso, mientras que el otro proyecto contiene el código necesario para desplegar un sistema utilizando un conector que permite emitir tramas ITS mediante BLE. El hardware de propósito general escogido para el proyecto es del fabricante Nordic Semiconductor.

## Marco de pruebas
 Mediante las pruebas de campo se quiere validar la tecnología. Se desarrollan dos escenarios: en el primero  los dos vehículos se encuentran estacionados, y el segundo está inspirado en el caso de uso Vulnerable Road User Warning de la especificación ITS.

<div width = 800 align = center><img src="img/escenarios.png" align = center alt="Escenarios" width = 700/></div>

Se emplean dos Thingy:91. se pueden programar conectándolos directamente a un ordenador y utilizando la aplicación Programmer de [nRF Connect para escritorio](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop). Otra opción es utilizar la placa de desarrollo nRF9160 conectando ambos dispositivos mediante un conector SWD. En ambos casos es necesario seleccionar el procesador que se quiere programar mediante el switch que presenta cada uno en el lateral.

<div width = 800 align = center><img src="img/thingy91.png" align = center alt="Thingy:91" width="200"/></div>

El código está disponible en la carpeta [Pruebas](Pruebas). Tanto el receptor como el transmisor ejecutan de manera simultánea dos aplicaciones, una en el nRF52840 SoC y otra en el nRF9160 SiP.
- [Receptor](Pruebas/Receptor) contiene el código necesario para el dispositivo que hace de Observer:  [observ_config](Pruebas/Receptor/observ_config) contiene el código que se ejecuta en el SiP, y [observer](Pruebas/Receptor/observer) se ejecuta en el SoC.

- [Transmisor](Pruebas/Transmisor) contiene el código necesario para el dispositivo que hace de Advertiser:  [advert_config](Pruebas/Receptor/advert_config) contiene el código que se ejecuta en el SiP, y [advertiser](Pruebas/Receptor/advertiser) se ejecuta en el SoC.

## Sistema ITS

<div width = 800 align = center><img src="img/sistema.png" align = center alt="Sistema" width="600"/></div>

El código relativo al sistema ITS se encuentra en la carpeta [Conector](Conector). Por un lado contiene la pila ITS Vanetza, que debe estar presente tanto en el receptor como en el transmisor. De entre las aplicaciones implementadas se va a ejecutar aplicación its_ble, que solicita a la pila cada cierto tiempo la transmisión de una trama ITS con un mensaje Cooperative Awareness Message en su interior. La opción CMAKE BUILD_ITS_BLE compila esta aplicación. Otra opción interesante es ITS_BLE_LOG, que activa el modo verbose. Este modo es útil para conocer el funcionamiento de la pila a más bajo nivel.

Las opciones disponibles de la aplicación se pueden consultar utilizando la opción --help. En este ejemplo  la aplicación toma el rol de dispositivo receptor, se le asigna una dirección MAC personalizada, tiene la entidad de seguridad activada y además la transmisión se realiza cada segundo. También se puede indicar el puerto serie por el que se quiere enviar la información al módem BLE, siendo por defecto /dev/ttyACM0. 
```bash
sudo ./its_ble --mac-address 00:00:00:00:00:01 --rol rx --security certs --certificate ticket-rx.cert --certificate-key ticket-rx.key --cam-interval 1000
```

Los dos proyectos que corresponden al módem transmisor y receptor se pueden ejecutar en cualquier dispositivo de Nordic Semiconductor que soporte la pila USB y disponga de un módulo BLE, como por ejemplo el kit de desarrollo nRF52840.

<div width = 800 align = center><img src="img/nrf52840dk.jpg" align = center alt="nRF52840" width="300"/></div>

- La carpeta [its_ble_rx](Conector/its_ble_rx) contiene el código de un dispositivo BLE advertiser. Emite paquetes de advertising con la cadena "ITS" en el campo Shortened Local Name. En Manufacturer Specific almacena las tramas ITS recibidas a través del puerto serie desde la pila Vanetza.

- La carpeta [its_ble_tx](Conector/its_ble_tx) contiene el código de un dispositivo BLE observer. Escanea el medio en busca de paquetes de advertising que contengan la cadena "ITS" en el campo Shortened Local Name. La trama ITS almacenada en Manufacturer Specific se transmite a través del puerto serie a la pila Vanetza.




