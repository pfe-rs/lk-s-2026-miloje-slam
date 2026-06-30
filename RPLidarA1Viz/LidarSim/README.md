# STA KOJI FAJL RADI

### Trenutno su najbitniji fajlovi 7 i 8

### 1 Data Sim And Viz
Generise i vizulalizuje presetovane sumovite simulacija Lidar skena.

### 2 Robot Movement
Simulira i vizualizuje simuliranu, presetovanu putanju. Sumoviti podaci.

### 3 Interactive Robot Movement
Simulira i vizualizuje putanju koju korisnik zada. Sumoviti podaci.

### 4 Scan Matcher
Trenutno ne radi sta treba zato sto je podesen na realne skenove, inace treba da mecuje simuliarne podatke od ranije (sumovite verovatno).

### 5 Interactive Robot Movement No Noise
Isto kao 3. fajl, samo bez suma. 

### 6 Newer Matcher
Putanja i fajlovi lose zadati trenutno, ne radi. Noviji matcher simuliranih podataka.

### 7 Lidar Reader and Saver
Cita podatke sa lidara i gura ih u CSV, igrati se sa diagnostickim porukama. Realni podaci.

### 8 Lidar Scan Matcher
Matchuje podatke sa lidara i odredjuje vektor pomeraja. VIZUELIZACIJA ISKLJUCENA ZBOG PI. CTRL+F vis i otkomentarisati adekvatnu liniju u slucaju da zelis da vidis simulaciju.

### 9 Vector Extraction
Izvlaci vektor promene polozaja iz skenova lidara (odometrija).

### 10 Pixelized Map
Prosirenje prethodnog koda da skenove pretvara u pikselizovanu mapu koja ce kasnije biti prosledjena path planneru da odredi putanju Miloja.


# ROS-ovanje koda

### Citanje i pablishovanje skena
### Vector Extraction uzima sken i trazi vektor promene
### Pixelizovanje mape pikselizuje mapu i salje je takvu path planeru
### Path planer pronalazi na koji piksel treba da se ode
### Motion planer pronalazi kako otici tamo (koje komande)
### Opet se skenira

### Tester
Cita koji fajlovi su u kom folderu.
