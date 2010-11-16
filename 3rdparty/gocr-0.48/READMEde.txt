
This is the short german version for README.
Dies ist die deutsche Kurzinformation fuer README.

                      --- GOCR v0.38 ---

Worum gehts?
- OCR = optical character recognition = Schrifterkennung
- liest pnm, pbm, pgm, ppm, einige pcx und tga Bilddateien
   (auf un*x-systemen mit libpnm fast alle Formate)
- gibt text aus
- Dieses Programm ist herausgegeben unter GPL (General Public License).
   Das bedeutet unter anderem:  FREI FUER ALLE BENUTZER.

Wie kompilieren?
  
  gzip -cd gocr-0.38.tgz | tar xfv -   # auspacken
  cd gocr-0.38	  # Ordner wechseln
  ./configure     # Makefile anpassen
  make  	  # gcc/g++ sollte installiert sein


Wie starten?
  gocr -h       	# Kurz(!)-Hilfe
  gocr file.pbm		# minimale Argumente 
  gocr -v 1 -v 32 -m 4 file.pbm # zusaetzlich debug-Modus und Layout-Analyse
 Optionen (bitte auch das aktuelle Manual zu gocr (man gocr) ansehen!):
  [-i] name - Bilddatei (pnm,pgm,pbm,ppm,pcx), - fuer stdin
  -o name   - Ausgabefile (Umlenkung der Standardausgabe)
  -e name   - Ausgabefile (Umlenkung der Fehlerausgabe)
  -x name   - Fortschrittsanzeige (file, fifo oder fd=1..255)
  -p name   - Verzeichnis der optionalen Datenbasis (z.B.: ./db/)
  -l num    - Schwellenwert 0<160<=255
  -d num    - Schmutzgroesse (Entfernen von Schmutz, -1=autodetect)
  -s num    - Wortabstand in Punkten (0 = autodetect)
  -v num    - Mehr Infos  [summiert]
      1      mehr Informationen
      2      Groesse der Boxen anzeigen (siehe -c)
      4      Boxen anzeigen (siehe -c)
      8      Ausgabe der Muster nach Erkennung
     16      Zeilenerkennung ausgeben
     32      debug-Ausgaben outXX.pgm
  -c string - Liste der auszugebenen Zeichen (_ = nicht erkanntes Zeichen)
  -C string - Zeichenfilter (z.B.: hexdigits: 0-9A-Fx, nur ASCII)
  -m num    - Arbeitsarten, ~ = abschalten
      2      Datenbank nutzen (in Entwicklung)
      4      Layout-Analyse (zum Testen)
      8      ~ vergleiche nichterkannte Zeichen
     16      ~ Teile verklebte Zeichen
     32      ~ Kontextkorrektur
     64      Zeichen komprimieren
    130      Datenbank erweitern (unbekannte Zeichen, Interaktiv)
    256      OCR Engine abschalten, macht nur Sinn mit -m 2
 Beispiel: gocr -v 6 -v 32 -c _YV text1.pbm
 - Programm ist noch sehr langsam, bitte Geduld!
  
Was geht nicht?
- vieles ... siehe README

Wie koennen Sie helfen?
- Schicken Sie Bemerkungen und KLEINE Beispieldaten als .pbm.gz. 
- Geld oder einen neuen Notebook koennte ich auch brauchen (paypal).
  Ok, paypal ist nicht mehr was es war, also vergesst es und mangelnde
  Zeit zum programmieren ist jetzt das groessere Problem.
- Wie finden Sie das Programm, wie viele Fehler erzeugt es im Vergleich
  zu anderen OCR-Programmen?
 (bitte Anzahl Fehler, Zeichen und Programmversion angeben)
- Gute Ideen nehme ich gern entgegen
  (d.h. kleine Aenderungen mit grosser Wirkung).
- Lob und Tadel nehm ich ebenfalls gerne entgegen.

Bugs:
- Das Programm hat sicher viele Fehler, nur welche? (siehe README)

Letzte Neuerungen:
  http://jOCR.sourceforge.net

Authoren: (siehe README und AUTHORS)

                      --- HAVE FUN ---
