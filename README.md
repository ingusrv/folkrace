# Robota vadības panelis

Šis risinājums ļauj pārvaldīt robotu un pēc katra brauciena analizēt no robota saņemtos datus. Gadījumā, ja ar risinājumu vēlas dalīties, papildus ir izveidota lietotāju sistēma, kura ļauj šo risinājumu izmantot vairākiem cilvēkiem atsevišķi.

__Risinājuma funkcionalitāte:__
- Lietotāju/login sistēma, kura ierobežo piekļuvi risinājumam
- Administratori arī spēj pārvaldīt visus lietotājus
- Lietotājs var izveidot robota "profilu", kurā iespējams redzēt ar robotu saistītu informāciju
- Robots spēj savienoties un komunicēt ar serveri, izmantojot sistēmā īpaši izveidotu atslēgu
- Lietotājs vavr sākt un beigt robota braukšanu
- Robots pēc brauciena nosūta datus uz serveri, kurus serveris sakārto un atspoguļo sistēmā.
- Lietotājs pēc braucieniem datus spēj savā starpā salīdzināt, lai saprastu, kurš robota algoritms strādāja labāk un kāda algoritma konfigurācija ir labākā
- Lietotājs spēj apskatīt no algoritma izvadīto informāciju, kā arī dzēst konkrētus brauciena datus

__Risinājuma (pagaidu) ierobežojumi:__
- Lietotājus nevar rediģēt
- Robotam nevar izveidot jaunu savienošanās atslēgu
- Brauciena datus nevar izdēst
- Brauciena datu salīdzināšanai nav iespējas pašam izvēlēties grafika nosaukumu - tiek automātiski izveidots grafika nosaukums no robota nosaukuma un datuma
- Brauciena datus var salīdzināt tikai pēc vidējā FPS
- Lietotāju dati nav atsevišķi atdalīti
- Nav atsevišķas datubāzes priekš izstrādes režīma(dev) un parastā režīma(prod)

__Risinājumā izmantotās tehnoloģijas:__
- Node.js priekš web servera koda
- MongoDB priekš web servera datubāzes
- HTML, CSS un JavaScript priekš frontend koda
- Python priekš robota koda
- Dažādas npm un Python bibliotēkas (express.js, mongodb, websockets u.c.)


🚧 __Šis projekts vēljoprojām tiek izstrādāts__ 🚧

## Nepieciešamās programmas, lai palaistu projektu:
- [Node.js](https://nodejs.org/en) ^19.0.0
- [MongoDB](https://www.mongodb.com/) ^6.0.5-1
- [Python](https://www.python.org/) ^3.11.3

Vecākas programmu versijas arī iespējams strādā, bet tas nav testēts.

__Nepieciešamās Python bibliotēkas priekš robota programmas:__
- [websockets](https://websockets.readthedocs.io/en/stable/) ^11.0.3
- Jau ieinstalētās bibliotēkas:
    - argparse
    - json
    - time

## Kā uzstādīt projektu:
1. Klonē šo repozitoriju
2. Atver repozitoriju kādā terminālī (Command Prompt, PowerShell, Bash)
3. Dodies uz ``web`` folder
4. Instalē nepieciešamās npm bibliotēkas ar šo komandu:
```
npm install
```
5. Šajā pašā vietā izveido jaunu failu ar nosaukumu ``.env``
6. Izveidotajā failā aizpildi šādus mainīgos:
    - ``DATABASE_URI=`` - Saite, kuru izmanto, lai savienotos ar MongoDb datubāzi
    - ``ROOT_USERNAME=`` - Galvenā administratora lietotājvārds, ar kuru sākotnēji var ielogoties mājaslapā
    - ``ROOT_PASSWORD=`` - Galvenā administratora parole, ar kuru sākotnēji var ielogoties mājaslapā
    - ``PRIVATE_KEY=`` - Atslēga, kuru izmanto, lai autentificētu lietotājus mājaslapā
    - ``PROD_PORT=`` - Ports, kuru serveris izmanto parastajā izmantošanas jeb "production" režīmā
    - ``DEV_PORT=`` - Ports, kuru serveris izmanto izstrādes jeb "development" režīmā

## Kā palaist risinājuma web daļu:
1. Terminālī ieraksti šo komandu, lai palaistu risinājumu parastajā izmantošanas režīmā:
```
npm run prod
```
Vai šo kommandu lai palaistu izstrādes režīmā:
```
npm run dev
```
2. Ja vēlas pieslēgties mājaslapai no viena un tā paša datora, uz kura risinājums palaists, var uzspiest uz terminālī izvadīto saiti
```
server started at <saite>
```

## Kā palaist risinājuma robota daļu
1. Dodies uz ``robots`` folder
2. Atver termināli šajā vietā un ieraksti šādu komandu:
```
python robots.py -a <adrese> -p <ports> -k <key>
```
Kur:
- ``<adrese>`` vietā ieraksti web servera adresi
- ``<ports>`` vietā ieraksti web servera portu
- ``<key>`` vietā ieraksti no mājaslapas ``"robotu saraksts"`` sadaļā iegūto robota savienošanās atslēgu

Lai pēc tam izietu no robota programmas uzspied taustiņu kombināciju ``ctrl+c``
