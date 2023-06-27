var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    switch(event.data) {
        case "1":
            aquaponicsMode();
            break;
        case "2":
            transferMode();
            break;
        case "3":
            aquacultureMode();
            break;
        case "4":
            hydroponicsMode();
            break;
        case "5":
            relaysOffMode();
            break;
        default:
            aquaponicsMode();
            break;
    }

    }

window.addEventListener('load', onLoad);

function onLoad(event) {
initWebSocket();
initButton();
}

function initButton() {
    document.getElementById('aquaponicsButton').addEventListener('click', button1);
    document.getElementById('transferButton').addEventListener('click', button2);
    document.getElementById('aquacultureButton').addEventListener('click', button3);
    document.getElementById('hydroponicsButton').addEventListener('click', button4);
    document.getElementById('offButton').addEventListener('click', button5);
}

function button1(){
    websocket.send('mode1');
}

function button2(){
    websocket.send('mode2');
}

function button3(){
    websocket.send('mode3');
}

function button4(){
    websocket.send('mode4');
}

function button5(){
    websocket.send('mode5');
}

function aquaponicsMode() {
    document.getElementById('aquaponicsButton').innerHTML = "ON";
    document.getElementById('aquaponicsButton').style.backgroundColor ="#0e8b00"; //on color green

    document.getElementById('transferButton').innerHTML = "OFF";
    document.getElementById('transferButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('aquacultureButton').innerHTML = "OFF";
    document.getElementById('aquacultureButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('hydroponicsButton').innerHTML = "OFF";
    document.getElementById('hydroponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('offButton').innerHTML = "RELAYS ON";
    document.getElementById('offButton').style.backgroundColor ="#011f72";   //off color blue
}

function transferMode() {
    document.getElementById('transferButton').innerHTML = "ONGOING";
    document.getElementById('transferButton').style.backgroundColor ="#0e8b00"; //on color green

    document.getElementById('aquaponicsButton').innerHTML = "OFF";
    document.getElementById('aquaponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('aquacultureButton').innerHTML = "OFF";
    document.getElementById('aquacultureButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('hydroponicsButton').innerHTML = "OFF";
    document.getElementById('hydroponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('offButton').innerHTML = "RELAYS ON";
    document.getElementById('offButton').style.backgroundColor ="#011f72";   //off color blue
}

function aquacultureMode() {
    document.getElementById('aquacultureButton').innerHTML = "ON";
    document.getElementById('aquacultureButton').style.backgroundColor ="#0e8b00"; //on color green

    document.getElementById('aquaponicsButton').innerHTML = "OFF";
    document.getElementById('aquaponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('transferButton').innerHTML = "OFF";
    document.getElementById('transferButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('hydroponicsButton').innerHTML = "OFF";
    document.getElementById('hydroponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('offButton').innerHTML = "RELAYS ON";
    document.getElementById('offButton').style.backgroundColor ="#011f72";   //off color blue
}

function hydroponicsMode() {
    document.getElementById('hydroponicsButton').innerHTML = "ON";
    document.getElementById('hydroponicsButton').style.backgroundColor ="#0e8b00"; //on color green

    document.getElementById('aquaponicsButton').innerHTML = "OFF";
    document.getElementById('aquaponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('transferButton').innerHTML = "OFF";
    document.getElementById('transferButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('aquacultureButton').innerHTML = "OFF";
    document.getElementById('aquacultureButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('offButton').innerHTML = "RELAYS ON";
    document.getElementById('offButton').style.backgroundColor ="#011f72";   //off color blue
}

function relaysOffMode() {
    document.getElementById('offButton').innerHTML = "RELAYS OFF";
    document.getElementById('offButton').style.backgroundColor ="#0e8b00"; //on color green

    document.getElementById('aquaponicsButton').innerHTML = "OFF";
    document.getElementById('aquaponicsButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('transferButton').innerHTML = "OFF";
    document.getElementById('transferButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('aquacultureButton').innerHTML = "OFF";
    document.getElementById('aquacultureButton').style.backgroundColor ="#011f72";   //off color blue

    document.getElementById('hydroponicsButton').innerHTML = "OFF";
    document.getElementById('hydroponicsButton').style.backgroundColor ="#011f72";   //off color blue
}