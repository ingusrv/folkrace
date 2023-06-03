# folkrace

Robota vadības panelis, kas ļauj pārvaldīt robotus un apskatīt to braukšanas datus

🚧 __Šis projekts vēljoprojām tiek izstrādāts__ 🚧

## Nepieciešamās programmas:
- [Node.js](https://nodejs.org/en) ^19.0.0
- [MongoDB](https://www.mongodb.com/) ^6.0.5-1
- [Python](https://www.python.org/) ^3.11.3

Vecākas programmu versijas arī iespējams strādā, bet tas nav testēts.

### Nepieciešamās Python bibliotēkas priekš robota programmas:
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
