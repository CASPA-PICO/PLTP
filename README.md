<h1>Packeted Lossless Transmission Protocol</h1>
<p>La bibliothèque est compatible avec les projets <a href="https://platformio.org/">Platform IO</a> et a été testé des ESP32 modèles : <a href="https://www.dfrobot.com/product-2231.html">DFR0654-F</a> et <a href="https://www.dfrobot.com/product-1590.html">DFR0478</a><br/></p>
<h2>Fonctionnement global de la bibliothèque</h2>
<p>Les données sont transmises sous la forme de paquets.<br/>Chaque paquet contient un entête de taille fixe indiquant la taille du paquet ainsi le checksum des données du paquet, puis les données en elle-même sont transmise après l'entête.<br/>
Quand on envoie des données, le premier octet indique le type de données envoyé (heure, entête d'un fichier, contenu d'un fichier).<br/>
Lorsque l'on envoie l'heure on transmet donc le premier octet avec le type "heure" puis le nombre de <a href="https://fr.wikipedia.org/wiki/Heure_Unix">l'heure au format Unix</a>.<br/>
Lorsque l'on envoie un fichier (mesures faites par le capteur), on envoie d'abord un paquet avec une donnée de type "entête d'un fichier" qui contient le nom du fichier ainsi que sa taille totale. Puis on transmet le contenu du fichier sous la forme de plusieurs paquets de données de type "contenu d'un fichier" en fonction de la taille totale du fichier.<br/>
Lors de la réception d'un paquet de données, on compte le checksum de l'entête des données avec le checksum qu'on calcule sur les données reçues afin de garantir l'intégrité du message.<br/>
</p>
