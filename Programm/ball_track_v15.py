#--------------------------------------
#
#   IDPA Rollende-Kugel
#   Autor: Thomas Zwicker
#   Datum: 23.01.2024
#   Version: 0.16
#
#--------------------------------------

# Bibliotheken Importieren
import cv2
import numpy as np
import time
import serial
import matplotlib.pyplot as plt
import threading

# Bild
Bild_x = 0
Bild_y = 0
Bild_höhe = 480
Bild_breite = 640
Bild_mitte_x = 320
Bild_mitte_y = 240

# Zustand
x_soll = 0
y_soll = 0
x_ist = 0 
y_ist = 0
x_alt = 0
y_alt = 0

# PID
PID_I_x = 0
PID_I_y = 0
PID_D_alt_x = 0
PID_D_alt_y = 0

PID_f_f = 0.15
PID_f_d = 4
PID_f_I = 0.15

Regler_zeit = 0
regeln = 0

# Motoransteuerung
serial1 = serial.Serial("/dev/ttyUSB0", 115200)
Pause = 0.0025
Min_wert_Motor = 80
Max_wert_Motor = 175

FPS = 0
FPS_counter = 0
FPS_timer = 0

schrift = cv2.FONT_HERSHEY_SIMPLEX

run = True # Hauptschleife

taste_alt = 0
wort = ""
shift = False
modus = ""

g_x = []
g_d = []
g_i = []
g_p = []
g_PID = []
fig = 0

# Buffer
pufer_list_x = []
pufer_list_y = []
pufer_n = 0
pufer_länge = 3
while pufer_n < pufer_länge:
    pufer_list_x.append(0)
    pufer_list_y.append(0)
    pufer_n = pufer_n + 1

def Kamera_einlesen():
    ret, Bild = Video.read()
    if not ret: print("Kamera nicht Bereit!")

    zuschnitt = Bild[Bild_y:Bild_y + Bild_höhe, Bild_x:Bild_x + Bild_breite]

    GrauBild = cv2.cvtColor(zuschnitt, cv2.COLOR_BGR2GRAY)
    BlurBild = cv2.GaussianBlur(GrauBild, (17, 17), 0)
    _, SchwelleBild = cv2.threshold(BlurBild, 127, 255, cv2.THRESH_BINARY) 

    return(zuschnitt, SchwelleBild)

def Nullen(Bild, Arbeitsbild):
    global Bild_x
    global Bild_y
    global Bild_höhe
    global Bild_breite
    global Bild_mitte_x
    global Bild_mitte_y

    # Konturen suchen
    Konturen, _ = cv2.findContours( 
        Arbeitsbild, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    i = 0 # setze Index auf 0

    # Liste mit allen Konturen
    for Kontur in Konturen: 

        # Erste Kontur wird übersprngen (Rand der Kamera)
        if i == 0: 
            i = 1
            continue

        # Die Form grob bestimmen
        approx = cv2.approxPolyDP(Kontur, 0.01 * cv2.arcLength(Kontur, True), True) 
        
        # Form zeichnen
        cv2.drawContours(Bild, [Kontur], 0, (0, 0, 255), 5) 

        try:
            # Mittelpunkt berechnung 
            M = cv2.moments(Kontur) 
            if M['m00'] != 0.0: 
                x = int(M['m10']/M['m00']) 
                y = int(M['m01']/M['m00']) 

            # Ecken Zählen und Auswerten
            if len(approx) == 3: 
                cv2.putText(Bild, "Dreieck", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 

            elif len(approx) == 4: 
                cv2.putText(Bild, "Viereck", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                if 240 < x < 400 and 120 < y < 240:
                    Bild_x, Bild_y, Bild_breite, Bild_höhe = cv2.boundingRect(approx)
                    Bild_x -= 10
                    Bild_y -= 10
                    Bild_breite += 20
                    Bild_höhe += 20
                    Bild_mitte_x = Bild_breite/2
                    Bild_mitte_y = Bild_höhe/2
                    print(Bild_mitte_x," : ",Bild_mitte_y,)
                    print(Bild_x, Bild_y, Bild_breite, Bild_höhe)
                    bild = Bild[Bild_y:Bild_y + Bild_höhe, Bild_x:Bild_x + Bild_breite]
                    #cv2.imshow('crop', bild)
                    

            elif len(approx) == 5: 
                cv2.putText(Bild, "Pentagon", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 

            elif len(approx) == 6: 
                cv2.putText(Bild, "Hexagon", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            else:
                cv2.putText(Bild, "Kreis", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        except:
            pass

    return(Bild)

def Kugel(Bild, Arbeitsbild):
    Kreise = cv2.HoughCircles(Arbeitsbild, cv2.HOUGH_GRADIENT, 1.5, 10,
                              param1=60, param2=30, minRadius=5, maxRadius=100)

    if Kreise is not None:
        Kreise = np.uint16(np.around(Kreise))
        #print("Zweitens: ", Kreise)

        if Kreise is not None:
            try:
                Kreis = str(Kreise)
                if Kreis.find("[[[ ") == 0:
                    Kreis = Kreis.removeprefix("[[[ ").removesuffix("]]]").replace("  "," ").split(" ")
                else:
                    Kreis = Kreis.removeprefix("[[[").removesuffix("]]]").replace("  "," ").split(" ")
                x = int(Kreis[0])
                y = int(Kreis[1])
                Durchmeser = int(Kreis[2])
                #print("xy:", x, y, Durchmeser)
                cv2.circle(Bild, (x, y), 2, (0,100,100), 3)
                cv2.circle(Bild, (x, y), Durchmeser,(255,0,255),3)

                x = (x - Bild_mitte_x)*(1)
                y = (y - Bild_mitte_y)*(-1)

                return(x, y, Bild)
            
            except:
                print("error")
                l = Kreise[0]
                for n in l:
                    print(n)

                print(Kreise)
    
    return(x_ist, y_ist, Bild)

def eingabe():
    global run, modus
    global PID_f_d, PID_f_f, PID_f_I

    if modus == "i":
        PID_f_I = float(wort)
        modus = ""
    elif modus == "f":
        PID_f_f = float(wort)
        modus = ""
    elif modus == "d":
        PID_f_d = float(wort)
        modus = ""
    elif len(modus) > 0:
        modus = ""

    if wort == "q":
        run = False
    elif wort == "i":
        modus = "i"
    elif wort == "f":
        modus = "f"
    elif wort == "d":
        modus = "d"
    
def Video_eingabe(taste):
    global wort, taste_alt, shift
    global x_soll, y_soll

    if taste != taste_alt and taste != -1:
        if taste == 225 or taste == 226:
            shift = True
            return()
        print(chr(taste), taste)
        if taste == 13:
            print(wort)
            eingabe()
            wort = ""
        elif taste == 8:
            wort = wort[:len(wort)-1]
        elif taste == 81:
            x_soll -= 5
        elif taste == 82:
            y_soll += 5
        elif taste == 83:
            x_soll += 5
        elif taste == 84:
            y_soll -= 5
        elif shift:
            wort = wort + chr(taste).upper()
            shift = False
        else:
            wort = wort + chr(taste)

    taste_alt = taste

def Anzeige(Bild, Arbeitsbild):
    
    hintergrund = np.zeros((900,1800, 3), dtype=np.uint8)
    hintergrund = cv2.putText(hintergrund, "Ist: x: {}  y: {}".format(x_ist,y_ist), (50,50), schrift, 1, (255,0,0), 2, cv2.LINE_AA)
    hintergrund = cv2.putText(hintergrund, "Soll: x: {}  y: {}".format(x_soll,y_soll), (50,100), schrift, 1, (255,0,0), 2, cv2.LINE_AA)
    hintergrund = cv2.putText(hintergrund, "FPS: {}".format(FPS), (50,150), schrift, 1, (255,0,0), 2, cv2.LINE_AA)
    hintergrund = cv2.putText(hintergrund, "M1: {}  M2: {} M3: {}".format(m_1,m_2,m_3), (50,200), schrift, 1, (255,0,0), 2, cv2.LINE_AA)

    if len(wort)> 0 or len(modus) > 0:
        hintergrund = cv2.putText(hintergrund, ">>> {}: {}".format(modus,wort), (50,350), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.LINE_AA)

    hintergrund = cv2.putText(hintergrund, "PID: F: {}  I: {} D: {}".format(float(PID_f_f), float(PID_f_I), float(PID_f_d)), (50,300), schrift, 1, (255,0,0), 2, cv2.LINE_AA)
    #hintergrund = cv2.putText(hintergrund, "PID_y: P: {}  I: {} D: {}".format(int(y_korigiert), int(PID_I_y_vis), int(PID_D_alt_y)), (50,350), schrift, 1, (255,0,0), 2, cv2.LINE_AA)
    

    Arbeitsbild = cv2.cvtColor(Arbeitsbild, cv2.COLOR_BGR2RGB)
    Arbeitsbild = cv2.resize(Arbeitsbild, (480, 480))
    hintergrund[420:900, 0:480, :] = Arbeitsbild
    hintergrund =cv2.rectangle(hintergrund, (0, 900), (480, 420), (255,0,0),10)

    Bild = cv2.resize(Bild, (900, 900))
    hintergrund[0:900, 900:1800, :] = Bild
    hintergrund =cv2.rectangle(hintergrund, (900, 0), (1800, 900), (255,0,0),10)



    cv2.imshow("Ruhende Kugel", hintergrund)

    taste_cv = cv2.waitKey(1)
    try:
        Video_eingabe(taste_cv)
    except:
        pass


    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #        run = False

def Korektur(z):
    z = z * Pixel_korrektur
    if z > 150:
        z = 150
    if z < -150:
        z = -150
    return(z)

def kontrolle(n):
    if n < Min_wert_Motor:
        return(Min_wert_Motor)
    elif n > Max_wert_Motor:
        return(Max_wert_Motor)
    else:
        return(n)
    
def analog_send(m1, m2, m3):
    m1 += 2
    m3 -= 4 # korrektur

    m1 = kontrolle(m1)
    m2 = kontrolle(m2)
    m3 = kontrolle(m3)

    serial1.write(b"500")
    time.sleep(Pause)
    m1 = bytes(str(m1), "utf-8")
    serial1.write(m1)
    time.sleep(Pause)
    m2 = bytes(str(m2), "utf-8")
    serial1.write(m2)
    time.sleep(Pause)
    m3 = bytes(str(m3), "utf-8")
    serial1.write(m3)

def regler(x, i, p_alt):
    
    d = (x - p_alt) * PID_f_d

    i = i + x
    i_f = i * PID_f_I*0.1

    # Faktor | P + D + I 
    PID = PID_f_f * (x + d + i_f) #0.08


    #print("pid y:",regeln, x, d, i_f, PID)

    
    if PID > 40: PID = 40
    if PID < -40: PID = -40

    PID = int(np.arcsin(PID/50)*0.9*57.3) # umrechnung von Kreisbewegung zu Linearbewegung
    #print(PID)
    return(PID, i, d, i_f)

def Motor_steuerung(x,y):
    m1x = 128 + x
    m2x = 128 + x #invertiert!
    m3x = 128 - x

    m1y = 128
    m2y = 128 + y #invertiert!
    m3y = 128 + y

    m1 = int((m1x + m1y)/2)
    m2 = int((m2x + m2y)/2)
    m3 = int((m3x + m3y)/2)

    analog_send(m1, m2, m3)

    return(m1,m2,256 - m3)

def setup_graf():
    global linie_p, linie_i, linie_d, linie_PID, fig
    n = 1
    while len(g_x) < 200:
        g_x.append(n)
        g_d.append(0)
        g_i.append(0)
        g_p.append(0)
        g_PID.append(0)
        n += 1

    plt.ion()

    fig = plt.figure() 
    ax = fig.add_subplot(111)
    linie_p, = ax.plot(g_x, g_p, label = "P")
    linie_i, = ax.plot(g_x, g_i, label = "I")
    linie_d, = ax.plot(g_x, g_d, label = "D")
    linie_PID, = ax.plot(g_x, g_PID, label = "PID")


    plt.axhline(0, color='black')
    #plt.axvline(0, color='black')

    ax.set_ylim(-150,150)
    plt.legend()

def graf():
    g_p.append(round(x_korigiert, 2))
    g_p.pop(0)
    #linie_p.set_ydata(g_p)

    g_i.append(round(PID_I_x_vis, 2))
    g_i.pop(0)
    #linie_i.set_ydata(g_i)

    g_d.append(round(PID_D_x, 2))
    g_d.pop(0)
    #linie_d.set_ydata(g_d)

    g_PID.append(round(x, 2))
    g_PID.pop(0)
    #linie_PID.set_ydata(g_PID)

    #fig.canvas.draw() 
    #fig.canvas.flush_events() 

def zeichnen():
    print("graf setup")
    while run:
        linie_p.set_ydata(g_p)
        linie_i.set_ydata(g_i)
        linie_d.set_ydata(g_d)
        linie_PID.set_ydata(g_PID)
        fig.canvas.draw() 
        fig.canvas.flush_events()
        time.sleep(1)

def puffer(x, pufer_list):
    pufer_list.pop(0)
    pufer_list.append(x)
    return(round(sum(pufer_list)/pufer_länge), pufer_list)

# Init
Video = cv2.VideoCapture(2) # Kamera starten
analog_send(128, 128, 128)
time.sleep(2)

while Bild_x == 0:
        Bild, Arbeitsbild = Kamera_einlesen()
        Bild = Nullen(Bild, Arbeitsbild)
        cv2.imshow("Nullen", Bild)
        cv2.waitKey(1)

max_pixel = (Bild_breite + Bild_höhe)/4 # /2 mittelwert /2 halbe höhe
Pixel_korrektur = 150 / max_pixel

setup_graf()
threading.Thread(target=zeichnen, daemon=True).start()

try:
    while run:
        Bild, Arbeitsbild = Kamera_einlesen()

        x_ist, y_ist, Bild = Kugel(Bild, Arbeitsbild)

        x_korigiert = Korektur(x_ist - x_soll)
        y_korigiert = Korektur(y_ist - y_soll)


        x, PID_I_x, PID_D_x, PID_I_x_vis = regler(x_korigiert, PID_I_x, PID_D_alt_x)
        y, PID_I_y, PID_D_y, PID_I_y_vis = regler(y_korigiert, PID_I_y, PID_D_alt_y)

        m_1, m_2, m_3 = Motor_steuerung(x,y)

        FPS_counter += 1
        if FPS_timer + 1 < time.time():
            FPS = FPS_counter
            FPS_counter = 0
            FPS_timer = time.time()
            print("x, pid_alt",x_korigiert, PID_I_x, PID_D_alt_x)

        PID_D_alt_x, pufer_list_x = puffer(x_korigiert, pufer_list_x)
        PID_D_alt_y, pufer_list_y = puffer(y_korigiert, pufer_list_y)
        
        #PID_D_alt_x = x_korigiert
        #PID_D_alt_y = y_korigiert

        graf()

        Anzeige(Bild, Arbeitsbild)


except KeyboardInterrupt:
    print("Programm beendet")
finally:
    Video.release()
    cv2.destroyAllWindows()
    time.sleep(1)
    analog_send(128, 128, 128)


