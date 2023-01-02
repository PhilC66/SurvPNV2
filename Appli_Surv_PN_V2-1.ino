/*

	Philippe CORBEL
	07/12/2017

	Telesurveillance PN V2
  V2-17 02/01/2023
    Sur reception SIRENE lancement ActivationSonnerie() plutot que commande de la Sirene en direct
    afin de respecter les tempo Sonnerie, eviter sonnerie trop longue

  IDE 1.8.19, AVR boards 1.8.6, PC fixe, Adafruit_FONA 1.3.106 modif PhC
	Le croquis utilise 40758 octets (16%), 1717 octets (20%) de mémoire dynamique
  IDE 1.8.19, AVR boards 1.8.6, Raspi fixe, Adafruit_FONA 1.3.106 modif PhC
	Le croquis utilise 40732 octets (16%), 1691 octets (20%) de mémoire dynamique

  V2-16 19/01/2022 installé PN64, 25/01/2022 PN56, PN62
    entree Alarme Secteur inversée sur PN64
    parametrage entrée Alarme Secteur selon Id 
  IDE 1.8.16, AVR boards 1.8.4, PC fixe, Adafruit_FONA 1.3.106 modif PhC
	Le croquis utilise 40764 octets (16%), 1717 octets (20%) de mémoire dynamique

	IDE 1.8.16, AVR boards 1.8.1, Raspberry
	Le croquis utilise 40738 octets (16%), 1691 octets (20%) de mémoire dynamique

	V2-15 15/12/2021 installé PN64

  IDE 1.8.16, AVR boards 1.8.4, PC fixe, Adafruit_FONA 1.3.106 modif PhC
	Le croquis utilise 40568 octets (15%), 1710 octets (20%) de mémoire dynamique

	IDE 1.8.16, AVR boards 1.8.1, Raspberry
	Le croquis utilise 40542 octets (15%), 1684 octets (20%) de mémoire dynamique

  mesure temps fermeture PN
  stockage coeff calibration en EEPROM séparément de config
  seconde entree hard en parallele pour Alarme Secteur
  suppression des interrupts sur DFV et CFV
  boucle acquistion 15s -> 10s
  divers corrections

  V2-14 31/08/2020 installé PN56 et PN62 25/03/2021
  ajouté sur message ST apres tension batterie OK/KO
  
  V2-13 07/05/2020 installé 19/05/2020 PN56 et PN62
  !!!!! Version carte SIM sans codePIN !!!!!
  suppression verif reseau

  V2-12 18/11/2019 installé PN56 et 62 le 04/02/2020
  1 - mise a jour de MAJHEURE pour eviter risque de blocage
      a reception du sms MAJHEURE recuperation heure du sms et maj Heure SIM et carte
      utilisation version Adafruit_FONA 1.3.106 modif PhC
  2 - nouveau magic, nouveau parametre par defaut

	V2-12 25/06/2019 pas encore installé
	1 - Bug sur PC Portable pas de remise à jour ou mise à jour incomplete EEPROM si nouveau magic
			Augmentation delay apres lecture EEPROM 500ms
	2 - suppression resetsim dans majheure
	3 - ajout dateheure dans tous les messages
	4 - envoie message fermeture PN en periode nuit
	5 - Creation message FALARME retourne etat des fausses alarmes meme sans Alarme en cours

	V2-11(ter) installé PN62 et PN56 le 18/04/2018
	1- ajout commande MAJHEURE, effectue "reset soft" SIM800 et lance mise a l'heure
	2- Mesure batterie 8V, nouveau numero magic, parametre Batterie2 = true, calibration en double
		 message BATTERIE2=ON/OFF
	3- lancement sirene et SMS directement sans attendre prochaine boucle acquisition

	V2-11bis pas encore installé 07/12/2017
	1- Correction bug blocagealarme voir descriptif (Bug 20171207.txt)
	2- forcer remise à l'heure à chaque reception PSUTTZ sans verif année
	3- bug a corriger sur reception message Silence OFF commande inverseée avec ON
	4- ajouter commande pour recuperer IMEI

	V2-11 pas encore installé 24/07/2017
	ajout calibration mesure tension
	nouveau numero magic
	Envoi Sonnerie/Sirene sur demande

	V2-10 installé PN56 03/07/2017

	ajout depuis V1
	passage sur Arduino MEGA 2560
	detection Intrusion, lancement Sonnerie du PN avec tempo
	Detection Intrusion valide(IntruAuto) entre Hsoir et Hmatin desactivé dans la journée cause capteur PIR
	Detection Barrieres fermées
	Lecture Heure Réseau
	Signe de Vie à heure fixe paramétrable
	gestion message en local sans sms
	gestion mise à l'heure automatique
	Gestion fausses Alarmes PIR id Autorail
  correction bug affichage Vbatt si decimale<10
	Attention le Detecteur PIR
	il faut que le timer de maintien de la sortie
	soit regler au plus rapide environ 2.5/3s

	EEPROM
	adrr=0	structure config. enregistre tous les parametres en EEPROM

	Librairie TimeAlarms.h modifiée
	#define dtNBR_ALARMS 10 (ne fonctionne pas avec 9)  6 à l'origine nombre d'alarmes RAM*11 max is 255

	Ajout dans la librairie Adafruit_FONA.h
	fonction lire Nom Expediteur du SMS si existe dans Phone Book:
															fona.getSMSSendername(slot, nameIDbuffer, 14)
	fonction lecture entrée du Phone Book :
															fona.getPhoneBookName(ligne, replybuffer,14) 14 max
															fona.getPhoneBookNumber(ligne, replybuffer,13) 40max
	fonction lire le Nom de l'Operateur
															fona.getNetworkName(replybuffer, 15);
	fonction lire etat carte SIM si Ready
															fona.getetatSIM() = true si Ready
  fonction lire date du sms
                              fona.getSMSdate()

*/

/* test seulement */
char 		receivedChar;
bool    newData = false;
String 	demande;
/* test seulement */

String ver = "V2-17";

#include <Adafruit_FONA.h>			// gestion carte GSM Fona SIM800
#include <EEPROM.h>							// variable en EEPROM
#include <EEPROMAnything.h>			// variable en EEPROM
#include <Time.h>								// gestion Heure
#include <TimeAlarms.h>					// gestion des Alarmes
#include <avr/wdt.h>						// watchdog uniquement pour Reset

/*  FONA_RX       2	==>     mega14 TX3
		FONA_TX       3	==>     mega15 RX3
		Entree Tension Batterie  A5 24V
		Entree Tension Batterie2 A4  8V */
#define Ip_AnalBatt1  5				//	entree analogique mesure tension	V2-11ter
#define Ip_AnalBatt2  4				//	entree analogique mesure tension	V2-11ter
#define FONA_RST      4					//	Reset SIM800
#define FONA_RI       6					//	Ring SIM800
#define Ip_PIR        18				//	Entrée detecteur PIR mouvement (7 avant)
#define Ip_AlaBatt    9					//	Entrée Alarme Batterie
#define Ip_AlaSecteur 10				//	Entrée Alarme Secteur PN56 et PN62 0=Alarme
#define Ip_AlaSecteur2 52				//	Entrée Alarme Secteur (plus facile à cabler) PN64 1=Alarme
#define S_Son         26				//	Sortie commande Sonnerie

// numero a definir pour mega, Interrupt 2, 3, 18, 19, 20, 21
#define DFV		        20				// Entree mesure Tension Cde Feux Vert1 IRQ
#define CFV		        21				// Entree mesure Tension Cde Feux Vert2 IRQ
#define S1		        32				// Sortie 1 Opto
#define S2		        34				// Sortie 2 Opto
#define S3		        30				// Sortie 3 Opto
#define S4		        28				// Sortie 4 Opto	
#define E1		        22				// Entree 1 Opto position Br1
#define E2		        24				// Entree 2 Opto position Br2
#define led						38				// Sortie Led Verte clignotante
#define led_PIR				46				// Sortie Voyant indic PIR

/*
	Entrée A5 Mesure Tension Batterie 24V R33k/3.3k => 30V/2.73V
	Vref externe 2.842V R5.1k entre 3.3V et Vref
*/

char Telbuff[14];								//	Buffer Numero tel
String fl = "\n";								//	saut de ligne SMS

String Id ;											// Id du PN sera lu dans EEPROM

char replybuffer[255];					// buffer de reponse FONA

// #include <SoftwareSerial.h>			// liaison serie FONA SIM800
// SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
// SoftwareSerial *fonaSerial = &fonaSS;
HardwareSerial *fonaSerial = &Serial3;	// liaison serie FONA SIM800

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

String  message;											//	Texte des SMS envoyé en reponse

char    fonaInBuffer[64];      				//	for notifications from the FONA

bool FlagAlarmeBatt 					= false;	//	Alarme Batterie
bool FlagLastAlarmeBatt 			= false;	//	verification derniere alarme

bool FlagAlarmeSect 					= false;	//	Alarme Secteur
bool FlagLastAlarmeSect 			= false;

bool FlagAlarmeTension 			= false;	  //	Alarme tension Batterie
bool FlagLastAlarmeTension 	= false;

bool FlagAlarmeIntrusion 		= false;	  //	Alarme Intrusion detectée
bool FlagLastAlarmeIntrusion = false;

bool FlagReset 							= false;	    //	Reset demandé=True
long    VBatterie1;												//	Tension Batterie PN 24V (22-29)
long    VBatterie2;												//	Tension Batterie PN  8V (7.3-9.6)

byte    Ntwk_dcx = 0;											//	compteur deconnexion reseau

volatile bool FlagDFV = false;		/* Demande Feux Verts true si PN actif,
																		 pas de mesures des barrieres possible
																		 présence tension 24V, id pour CFV */
volatile bool FlagCFV = false;		// Commande Feux Verts, apres DFV si les 2 Barrieres sont fermées

volatile uint32_t rebond1 = 0;
volatile uint32_t rebond2 = 0;
volatile uint32_t rebond3 = 0;
uint32_t timedebutFermeture = 0;  // Debut fermeture PN
uint32_t timefinFermeture = 0;    // Fin fermeture PN
uint32_t dureeFermeture = 0;      // Durée Fermeture PN

bool Br1 							= false;			// true si barrieres fermée
bool Br2 							= false;
bool FlagAlarmeBar 		= false;			// Alarme Barriere tombée = true
bool FlagLastAlarmeBar = false;
bool FirstSonn 				= false;			// Premier appel sonnerie
bool SonnMax   				= false;			// temps de sonnerie maxi atteint
bool ModeTest  				= false;			// mode test reduit temporairement la durée Sonnerie à 1s
byte CptTest   				= 12;					// décompteur en mode test si=0 retour tempo normale
//V2-11
int			CoeffTensionDefaut = 3100;	// Coefficient par defaut
bool    FlagCalibration = false;	  // Calibration Tension en cours
int     CoeffTension;							  // Coefficient calibration Tension V2-15
int			CoeffTension2;						  // Coefficient calibration Tension2 V2-15
//V2-11
bool TypePN = false;                // PN56,62=true,64=false defini entree et sens Alarme Secteur
struct config_t 										// Structure configuration sauvée en EEPROM
{
  int  magic		;									  // num magique
  long Ala_Vie ;									  // Heure message Vie, 8h matin en seconde = 8*60*60
  bool Intru   ;									  // Alarme Intrusion active
  bool IntruAuto;								    // Mode Alarme Intrusion automatique entre Hsoir et Hmatin
  long IntruFin;									  // Heure arret Alarme Intru Matin
  long IntruDebut;								  // Heure debut Alarme Intru Soir
  bool Silence ;									  // Mode Silencieux = true false par defaut
  bool Bar 	  ;									    // Alarme Barriere active
  bool Pos_PN 	;									  // envoie un SMS a chaque fermeture
  bool Pos_Pn_PB[10];						    // numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
  int  Dsonn 	;									    // Durée Sonnerie
  int  DsonnMax;									  // Durée Max Sonnerie
  int  Dsonnrepos;								  // Durée repos Sonnerie
  int  timecomptemax;	 					    // Temps de la boucle fausses alarme 1200 = 2mn
  int  Nmax ;				                // Nombre de fausses alarmes avant alarme
  char Idchar[11];								  // Id
  int  CoeffTension;							  // Coefficient calibration Tension	V2-11
  bool Batterie2;								    // presence Batterie2 8V            V2-11ter
  int	 CoeffTension2;						    // Coefficient calibration Tension2 V2-11ter
} config;
byte EEPROM_adresse[3] = {0, 100, 110};//config add=0, coeffbatt1 add=100, coeffbatt2 add=110
bool FlagTempoIntru 	= false;		  // memorise config.Intru au demarrage
bool FlagPNFerme 		= false;		    // true si PN fermé
bool FlagLastPNFerme = false;
bool FlagPIR					= false;
volatile int CptAlarme	 = 0;				//	compteur alarme avant filtrage
int 			 FausseAlarme  = 0;				//	compteur fausse alarme
int 			 timecompte		 = 0;    		//	comptage nbr passage dans loop compteur temps fausses alarmes

/* Identification des Alarmes*/
AlarmId FirstMessage;		// 0 tempo lancement premier message au demarrage
AlarmId loopPrincipale;	// 1 boucle principlae
AlarmId Svie;						// 2 tempo Signal de Vie
AlarmId MajH;						// 3 tempo mise à l'heure régulière
AlarmId TSonn;					// 4 tempo durée de la sonnerie
AlarmId TSonnMax;				// 5 tempo maximum de sonnerie
AlarmId TSonnRepos;			// 6 tempo repos apres maxi
AlarmId HIntruF;				// 7 Heure Fin Matin Alarme Intru
AlarmId HIntruD;				// 8 Heure Debut Soir Alarme Intru

//---------------------------------------------------------------------------
// void IRQ_DFV() {	// Tension presente sur commande des feux Verts, PAS UTILE
//   if (millis() - rebond1 > 20) {		// antirebond
//     rebond1 = millis();
//     FlagDFV = !digitalRead(DFV);	// true = pas de mesure position barrieres possible
//   }
// }

// void IRQ_CFV() {
//   if (millis() - rebond2 > 20) {	// antirebond
//     rebond2 = millis();
//     FlagCFV = !digitalRead(CFV);	// true = pas de mesure position barrieres possible
//   }
// }

void IRQ_PIR() {										// detecteur PIR
  if (config.Intru) {
    if (millis() - rebond3 > 20) {		// antirebond
      rebond3 = millis();
      CptAlarme ++;
    }
  }
}
//---------------------------------------------------------------------------
void setup() {
  rebond1 = millis();
  rebond2 = millis();
  rebond3 = millis();

  message.reserve(140);										// texte des SMS
  while (!Serial);
  Serial.begin(9600);											//	Liaison Serie PC ex 115200
  Serial.println(__FILE__);
  /* Lecture configuration en EEPROM */
  EEPROM_readAnything(EEPROM_adresse[0], config);
  Alarm.delay(500);	// Obligatoire compatibilité avec PC Portable V2-12
  int magic = 1234;
  if (config.magic != magic) {	// V2-11ter
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut */
    Serial.println(F("Nouvelle carte vierge !"));
    config.magic         = magic;	// V2-11ter
    config.Ala_Vie       = 25560;	// 7h06=25560
    config.Intru         = true;
    config.Silence       = false;
    config.IntruAuto     = true;
    config.IntruFin      = 21600; // 06h00 21600
    config.IntruDebut    = 75600; // 21h00 75600
    config.Bar           = true;
    config.Pos_PN        = false;
    config.Dsonn         = 200;
    config.DsonnMax      = 300;
    config.Dsonnrepos    = 11;
    config.timecomptemax = 900;
    config.Nmax          = 1;
    String temp          = "TPCF_PN000";
    temp.toCharArray(config.Idchar, 11);
    config.CoeffTension  = CoeffTensionDefaut;			// valeur par defaut	V2-11
    config.CoeffTension2 = CoeffTensionDefaut;			// valeur par defaut	V2-11ter
    config.Batterie2     = false;										// presence Batterie2	V2-11ter

    for (int i = 0; i < 10; i++) {
      config.Pos_Pn_PB[i] = 0;
    }
    config.Pos_Pn_PB[1] = 1;	// le premier numero du PB par defaut
    int longueur = EEPROM_writeAnything(EEPROM_adresse[0], config);	// ecriture des valeurs par defaut long=55
  }

  Id  = String(config.Idchar);
  Id += fl;

  Serial.print("Magic="), Serial.println(config.magic);
  Serial.print("Ala_Vie="), Serial.println(config.Ala_Vie);
  Serial.print("Intru="), Serial.println(config.Intru);
  Serial.print("IntruFin="), Serial.println(config.IntruFin);
  Serial.print("IntruDebut="), Serial.println(config.IntruDebut);
  Serial.print("Bar="), Serial.println(config.Bar);
  Serial.print("Dsonn="), Serial.println(config.Dsonn);
  Serial.print("DsonnMax="), Serial.println(config.DsonnMax);
  Serial.print("Drepos="), Serial.println(config.Dsonnrepos);
  Serial.print("Idchar="), Serial.println(config.Idchar);
  Serial.print("Id="), Serial.println(Id);
  Serial.print("Pos_PN="), Serial.println(config.Pos_PN);
  Serial.print(F("Batterie2 ")), Serial.println(config.Batterie2);
  Serial.print("Phone Book=");
  for (int i = 0; i < 10; i++) {
    Serial.print(config.Pos_Pn_PB[i]), Serial.print(",");
  }
  Serial.println("");

  sensAlarmeSecteur();

  EEPROM_readAnything(EEPROM_adresse[1], CoeffTension);
  EEPROM_readAnything(EEPROM_adresse[2], CoeffTension2);
  Serial.print(F("CoeffTension1 ")), Serial.println(CoeffTension);
  Serial.print(F("CoeffTension2 ")), Serial.println(CoeffTension2);
  /* test */
  analogReference(EXTERNAL);// reference Analog 3.3V au travers de 5.1K  Vref=2.85V, 1023=31.32V
  /* test */
  pinMode(Ip_AlaBatt, INPUT_PULLUP);			// Entrée Alarme Batterie
  pinMode(Ip_AlaSecteur, INPUT_PULLUP);		// Entrée Alarme Secteur
  pinMode(Ip_AlaSecteur2, INPUT_PULLUP);	// Entrée Alarme Secteur2
  pinMode(Ip_PIR, INPUT_PULLUP);					// Entrée Detecteur PIR
  pinMode(S_Son, OUTPUT);									// Sortie commande Sonnerie
  digitalWrite(S_Son, LOW);								// Sortie Sonnerie OFF
  pinMode(DFV, INPUT_PULLUP);
  pinMode(CFV, INPUT_PULLUP);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, OUTPUT);
  pinMode(E1, INPUT_PULLUP);
  pinMode(E2, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(led_PIR, OUTPUT);

  digitalWrite(S1, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  digitalWrite(S4, LOW);
  digitalWrite(led, LOW);
  digitalWrite(led_PIR, LOW);

  FlagDFV = !digitalRead(DFV);
  FlagCFV = !digitalRead(CFV);

  /* Creation des Interruptions mesure tension Commande Feux Verts */
  // attachInterrupt(digitalPinToInterrupt(DFV), IRQ_DFV, RISING);
  // attachInterrupt(digitalPinToInterrupt(CFV), IRQ_CFV, RISING);

  Serial.print(F("Version : ")), Serial.println(ver);
  Serial.println(F("Lancement Application "));
  Serial.println(F("Initialisation Module GSM...."));

  fonaSerial->begin(9600);								//	Liaison série FONA SIM800 ex 4800
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print(F("Module IMEI: ")); Serial.println(imei);
  }
  Serial.println(F("FONA Ready"));
  Alarm.delay(4000);			// attendre carte SIM ready
  flushSerial();

  if (!fona.getetatSIM()) {	// Si carte SIM not READY, Envoyé PIN
    flushSerial();
    char PIN[5] = "1234";
    byte retries = 1;
    if (! fona.unlockSIM(PIN)) {
      Serial.println(F("Failed to unlock SIM"));
      retries++;
      Alarm.delay(1000);
      if (retries == 3) {
        goto sortie;					// 2 tentatives max
      }
    }
    else {
      Serial.println(F("OK SIM Unlock"));
    }
sortie:
    Alarm.delay(1000);				//	Attendre cx reseau apres SIM unlock
  }
  byte n;
  byte cpt = 0;
  do {												// boucle tant que reseau pas connecté
    Alarm.delay(2000);
    n = fona.getNetworkStatus();
    cpt ++;
    if (cpt > 10) break;				// sortie si 10 tentatives demarrage sans reseau
  } while (!(n == 1 || n == 5));	//	si pas connecté reseau doit etre 1 ou 5
  Serial.print(F("Network status "));
  Serial.print(n);
  Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  flushSerial();
  // Demande Operateur connecté
  fona.getNetworkName(replybuffer, 15);
  Serial.print(F("Operateur :")), Serial.println(replybuffer);

  message = "";
  read_RSSI();									// Niveau reseau
  Serial.println(message);
  message = "";

  MajHeure();										// Mise à jour Date et Heure depuis réseau

  /* parametrage des Alarmes */

  FirstMessage = Alarm.timerOnce(60, OnceOnly); // appeler une fois apres 60 secondes type=0
  /* test 5*/
  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);
  /* test 600*/
  MajH = Alarm.timerRepeat(3600, MajHeure);						// toute les heures  type=1
  Alarm.enable(MajH);

  TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo durée de la sonnerie
  Alarm.disable(TSonn);

  TSonnMax = Alarm.timerRepeat(config.DsonnMax, SonnerieMax); // tempo maximum de sonnerie
  Alarm.disable(TSonnMax);

  TSonnRepos = Alarm.timerRepeat(config.Dsonnrepos, ResetSonnerie); // tempo repos apres maxi
  Alarm.disable(TSonnRepos);

  Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie); // chaque jour type=3
  //Serial.print(F("Alarme vie =")),Serial.println(Alarm.read(Svie));
  Alarm.enable(Svie);


  HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);
  HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);
  Alarm.enable(HIntruD);
  Alarm.enable(HIntruF);

  //AIntru_HeureActuelle(); // armement selon l'heure

  if (config.Intru) {								// si Alarme Intru active, desactive pendant demarrage
    FlagTempoIntru = config.Intru;	// on memorise
    config.Intru = false;						// on desactive jusqu'a tempo demarrage 1mn
  }


  Serial.print(F("FreeRAM = ")), Serial.println(freeRam());

}	//fin setup
//---------------------------------------------------------------------------
void loop() {
  /* test seulement */
  recvOneChar();
  showNewData();
  /* test seulement */
  static bool timerlance = false;						  //	activation timer alarme
  if (rebond1 > millis()) rebond1 = millis();	// anti rebonds
  if (rebond2 > millis()) rebond2 = millis();
  if (rebond3 > millis()) rebond3 = millis();

  // Attente donnée en provenance ds SIM800
  //wdt_reset();
  String bufferrcpt;
  char* bufPtr = fonaInBuffer;	//handy buffer pointer
  if (fona.available())      		//any data available from the FONA?
  {
    byte slot = 0;            	//this will be the slot number of the SMS
    unsigned int charCount = 0;
    //Read the notification into fonaInBuffer
    do  {
      *bufPtr = fona.read();
      bufferrcpt += *bufPtr;
      Serial.write(*bufPtr);
      Alarm.delay(1);
    } while ((*bufPtr++ != '\n') && (fona.available()) && (++charCount < (sizeof(fonaInBuffer) - 1)));
    //Add a terminal NULL to the notification string
    *bufPtr = 0;
    // if (charCount > 1) {
    // Serial.print(F("Buffer ="));
    // Serial.println(bufferrcpt);
    // }
    // Si appel entrant on raccroche
    if ((bufferrcpt.indexOf(F("RING"))) == 0) {	// RING, Ca sonne
      //Serial.println(F("Ca sonne!!!!"));
      fona.hangUp();											// on raccroche
    }
    if ((bufferrcpt.indexOf(F("PSUTTZ"))) >= 0 ) { // V2-11bis rattrapage si erreur mise à la date
      //Serial.println(F("Relance mise à l'heure !"));
      MajHeure();	// mise à l'heure
    }
    // Scan the notification string for an SMS received notification.
    // If it's an SMS message, we'll get the slot number in 'slot'
    if (1 == sscanf(fonaInBuffer, "+CMTI: \"SM\",%d", &slot)) {
      traite_sms(slot);
    }
  }

  if (config.Intru && CptAlarme > 0) {		// Si alarme PIR
    if (!timerlance)timerlance = true;		// on lance le timer si pas deja fait
    timecompte ++;
    if (CptAlarme > config.Nmax && timecompte < config.timecomptemax) { // Alarme validée
      FlagPIR = true;
    }
    if (timecompte > config.timecomptemax || FlagPIR) { 	// remise à 0 du comptage apres 1mn
      timerlance = false; 							// on arrete le timer
      timecompte = 0;
      FausseAlarme += CptAlarme;
      CptAlarme = 0;
      Serial.print(F("fausse alarmes : ")), Serial.println(FausseAlarme);
      Acquisition();	//V2-11ter on lance Sirene et SMS directement sans attendre prochaine boucle
    }
  }
  if (digitalRead(Ip_PIR) && config.Intru) {
    digitalWrite(led_PIR, HIGH);	// allume led locale
  }
  else {
    digitalWrite(led_PIR, LOW);
  }
  static byte lastseconde = 0;
  if(second() % 2 == 0 && second() != lastseconde){
    VerifBarriere();  // verification etat des barrieres
    lastseconde = second();
  }
  Alarm.delay(10);

}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {
  // ************************ boucle acquisition  ***************************
  //	boucle acquisition 10s

  displayTime(false);

  // verification si toujours connecté au réseau
  // byte n = fona.getNetworkStatus();
  //Serial.print(F("netwkstatus=")),Serial.println(n);
  // if (n != 1 && n != 5) {
    // Ntwk_dcx++;
    // if (Ntwk_dcx > 20) { // 20x15s=5mn
      // Serial.println(F("Pas de reseau !"));
      // softReset();					//	redemarrage Arduino apres 5mn
    // }
  // }
  // else {
    // if (Ntwk_dcx > 0) Ntwk_dcx --;
  // }

  static byte nalaBatt = 0;					// compteur alarme consecutive
  if (!digitalRead(Ip_AlaBatt)) {
    nalaBatt++;
    if (nalaBatt == 4) {
      FlagAlarmeBatt = true;
      nalaBatt = 0;
    }
    Serial.println(F("Alarme Batterie"));
  }
  else {
    FlagAlarmeBatt = false;
    if (nalaBatt > 0)nalaBatt--;						//	efface progressivement le compteur
  }

  static byte nalaSecteur = 0;
  //if (!digitalRead(Ip_AlaSecteur) || !digitalRead(Ip_AlaSecteur2)) {
  if ((!digitalRead(Ip_AlaSecteur) && TypePN) || (digitalRead(Ip_AlaSecteur2) && !TypePN)) {
    nalaSecteur ++;
    if (nalaSecteur == 4) {
      FlagAlarmeSect = true;
      nalaSecteur = 0;
    }
    Serial.println(F("Alarme Secteur"));
  }
  else {
    FlagAlarmeSect = false;
    if (nalaSecteur > 0)nalaSecteur--;			//	efface progressivement le compteur
  }

  static byte nalaTension = 0;
  VBatterie1 = map(moyenneAnalogique(Ip_AnalBatt1), 0, 1023, 0, CoeffTension);//V2-11
  if (config.Batterie2) {
    VBatterie2 = map(moyenneAnalogique(Ip_AnalBatt2), 0, 1023, 0, CoeffTension2);//V2-11ter
  }
  // Serial.print(F("Tension2 = ")),Serial.println(VBatterie2);
  // Serial.print(F("coeff2 = ")),Serial.println(CoeffTension2);
  if ((VBatterie1 < 2200 || VBatterie1 > 2880) || (config.Batterie2 && (VBatterie2 < 733 || VBatterie2 > 960))) { //V2-11ter
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else {
    FlagAlarmeTension = false;
    if (nalaTension > 0)nalaTension--;			//	efface progressivement le compteur
  }

  // gestion du capteur coupé ou en alarme permanente
  // verif sur 3 passages consecutifs
  static byte nalaPIR = 0;
  if (config.Intru && digitalRead(Ip_PIR)) {	// lecture capteur PIR
    nalaPIR ++;
    if (nalaPIR > 3) {
      CptAlarme    = 1;
      FausseAlarme = 1000;
      FlagPIR = true;
      nalaPIR = 0;
    }
  }
  else {
    if (nalaPIR > 0) nalaPIR --;			//	efface progressivement le compteur
  }
  if (FlagPIR) {
    FlagAlarmeIntrusion = true;				// Si alarme intrusion active et intrusion detectée
    FlagPIR = false;
    ActivationSonnerie();							// activation Sonnerie
    Serial.print(F("Alarme Intrusion ")), Serial.println(config.Intru);
  }
  if (config.Intru){
    Serial.print(F("Compte tempo = ")), Serial.println(timecompte);
    Serial.print(F("Compte Alarme = ")), Serial.println(CptAlarme);
  }

  // verification nombre SMS en attente(raté en lecture directe)
  int8_t smsnum = fona.getNumSMS();
  Serial.print(F("Sms en attente =")), Serial.println (smsnum);

  if (smsnum > 0) {	// nombre de SMS en attente
    // il faut les traiter
    traite_sms(51);// demande traitement de tous les SMS en attente
  }
  else if (smsnum == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    softReset();					//	redemarrage Arduino
  }


  if (ModeTest) {
    CptTest --;				// en debut ModeTest =12 soit 12*15s = 3 mn
    Serial.print(F("Mode test, cpt=")), Serial.println(CptTest);
    Serial.print(F("durée sonn =")), Serial.println(Alarm.read(TSonn));
    if (CptTest == 0) {	// retour à tempo sonnerie normale
      Alarm.disable(TSonn);			// on arrete la tempo sonnerie
      TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo durée de la sonnerie
      //Alarm.enable(TSonn);
      ModeTest = false;
      Serial.print(F("fin de cycle, durée sonn =")), Serial.println(Alarm.read(TSonn));
    }
  }
  envoie_alarme();

  digitalWrite(led, HIGH);	// voyant
  Alarm.delay(100);
  digitalWrite(led, LOW);
  Serial.print(F("freeRAM = ")), Serial.println(freeRam());

  // *********************fin boucle acquisition  ***************************
}
//---------------------------------------------------------------------------
void traite_sms(byte slot) {	// traitement du SMS par slot
  /* il y a 50 slots dispo
  	si slot=51, demande de balayer tous les slots pour verification
  	si slot=99, demande depuis liaison seria en test, traiter sans envoyer de sms */

  Serial.print(F("slot: ")); Serial.println(slot);
  uint16_t smslen;
  String textesms;
  char callerIDbuffer[13];  //we'll store the SMS sender number in here
  char nameIDbuffer[15];  	//nom expediteur SMS si existe dans Phone Book
  byte i;
  byte j;
  bool sms = true;
  static int tensionmemo1 = 0;	//	memorisation tension batterie lors de la calibration V2-11
  static int tensionmemo2 = 0;	//	memorisation tension batterie lors de la calibration V2-11ter
  if (slot == 99) sms = false;
  if (slot == 51) { // demande de traitement des SMS en attente
    i = 1;
    j = 50;
  }
  else {
    i = slot;
    j = slot;
  }
  for (byte k = i; k <= j; k++) {
    //wdt_reset();
    slot = k;
    // /* Retrieve SMS sender address/phone number. */
    if (sms) {
      if (! fona.getSMSSender(slot, callerIDbuffer, 13)) {
        Serial.println(F("Didn't find SMS message in slot!"));
        continue;	//	Next k
      }
      fona.getSMSSendername(slot, nameIDbuffer, 14);
      Serial.print(F("Nom appelant:")), Serial.println(nameIDbuffer);
      fona.readSMS(slot, replybuffer, 250, &smslen);
    }
    textesms = String(replybuffer);
    Serial.print(F("texte du SMS=")), Serial.println(textesms);
    for (byte i = 0; i < textesms.length(); i++) {
      if ((int)textesms[i] < 0 || (int)textesms[i] > 127) { // caracteres accentués interdit
        goto sortir;
      }
    }
    if ((sms && String(nameIDbuffer).length() > 0) || !sms) { // nom appelant existant
      //Envoyer une réponse
      //Serial.println(F("Envoie reponse..."));
      messageId();
      if (!(textesms.indexOf(F("TEL")) == 0 || textesms.indexOf(F("tel")) == 0 || textesms.indexOf(F("Tel")) == 0)) {
        textesms.toUpperCase();		// passe tout en Maj sauf si "TEL"
        textesms.replace(" ", "");	// supp tous les espaces
      }
      else {

      }
      // Serial.print(F("texte du SMS txt =")), Serial.println(textesms);
      // Serial.print(F("texte du SMS char=")), Serial.println(replybuffer);
      if (textesms.indexOf(F("??")) == 0) {	//	Aide "??"
        message += F("Liste des commandes :");
        message += fl ;
        message += F("Etat PN  : ETAT/PN/ST");
        message += fl;
        message += F("Etat SYS : SYS");
        message += fl;
        message += F("Reset Alarme/Sys :RST");
        message += fl;
        message += F("Alarme Intrusion");
        message += fl;
        message += F("Intru ON/OFF/Auto");
        sendSMSReply(callerIDbuffer, sms);	// SMS n°1

        message  = Id;
        message += F("Param Sonnerie: SONN");
        message += fl;
        message += F("SONN=xx:yy:zz (s)");
        message += fl;
        message += F("Alarme Barriere ON/OFF");
        message += fl;
        message += F("Bar ON/OFF");
        message += fl;
        message += F("Message Vie");				//Heure du message signe de vie
        message += fl;
        message += F("Vie = 1-24 (H en s)");	// de 0 à 24H en secondes
        sendSMSReply(callerIDbuffer, sms);	// SMS n°2

        message  = Id;
        message += F("Nouvel Id : Id= (max 10c)");
        message += fl;
        message += F("List Num Tel:LST?");
        message += fl;
        message += F("Nouveau Num Tel: ");
        message += fl;
        message += F("Tel=+33612345678,") ;
        message += fl;
        message += F("Nom(max 14c)");
        sendSMSReply(callerIDbuffer, sms);	// SMS n°3
      }

      else if (textesms.indexOf(F("TEL")) == 0
               || textesms.indexOf(F("Tel")) == 0
               || textesms.indexOf(F("tel")) == 0) { // entrer nouveau num
        bool FlagOK = true;
        byte j = 0;
        String Send	= F("AT+CPBW=");	// ecriture dans le phone book
        if (textesms.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
          int i = textesms.substring(3).toInt();// recupere n° de ligne
          i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
          //Serial.print(F("Num ligne : ")),Serial.println(i);
          if (i < 1) FlagOK = false;
          Send += i;
          j = 5;
          // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
          if ( (i != 1) && (textesms.indexOf(F("efface")) == 5
                            || textesms.indexOf(F("EFFACE")) == 5 )) goto fin_tel;
        }
        else if (textesms.indexOf(char(61)) == 3) { // TEL= nouveau numero
          j = 4;
        }
        else {
          FlagOK = false;
        }
        if (textesms.indexOf("+") == j) {			// debut du num tel +
          if (textesms.indexOf(",") == j + 12) {	// verif si longuer ok
            String numero = textesms.substring(j, j + 12);
            String nom = textesms.substring(j + 13, j + 27);	// pas de verif si long<>0?
            Send += F(",\"");
            Send += numero;
            Send += F("\",145,\"");
            Send += nom;
            Send += F("\"");
          }
          else {
            FlagOK = false;
          }
        }
        else {
          FlagOK = false;
        }
fin_tel:
        if (!FlagOK) { // erreur de format
          //Serial.println(F("false"));
          messageId();
          message += F("Commande non reconnue ?");// non reconnu
          sendSMSReply(callerIDbuffer, sms);						// SMS non reconnu
        }
        else {
          Serial.println(Send);
          fona.println(Send.c_str());						//ecriture dans PhoneBook
          Alarm.delay(500);
          fona.println(F("AT+CMGF=1"));					//pour purger buffer fona
          Alarm.delay(500);
          messageId();
          message += F("Nouveau Num Tel: ");
          message += F("OK");
          sendSMSReply(callerIDbuffer, sms);
        }
      }
      else if (textesms == F("LST?")) {	//	Liste des Num Tel
        messageId();
        for (byte i = 1; i < 10; i++) {
          //wdt_reset();
          char name[15];
          char num[14];
          if (!fona.getPhoneBookName(i, name, 14)) { // si existe pas sortir
            //Serial.println("Failed!");// next i
            goto fin_i;
          }
          fona.getPhoneBookNumber(i, num, 13);
          message += String(i) + ":";
          message += String(num);
          message += "," + fl;
          message += String(name);
          message += fl;
          if ((i % 3) == 0) {
            sendSMSReply(callerIDbuffer, sms);// envoi sur plusieurs SMS
            //Serial.println(message);
            messageId();
          }
        }
fin_i:
        if (message.length() > Id.length()+20)sendSMSReply(callerIDbuffer, sms); // SMS final (V1.1)
        //Serial.println(message);
      }
      else if (textesms.indexOf(F("PN")) == 0 || textesms.indexOf(F("ETAT")) == 0 || textesms.indexOf(F("ST")) == 0) {	// "ETAT PN?"
        generationMessage();
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("SYS")) == 0) {					//	Etat Systeme
        messageId();
        flushSerial();
        fona.getNetworkName(replybuffer, 15);		// Operateur
        Serial.println(replybuffer);
        flushSerial();
        byte n = fona.getNetworkStatus();
        if (n == 5) {														// Operateur roaming
          message += F("rmg, ");							// roaming
          message += replybuffer + fl;
        }
        else {
          message += replybuffer + fl; 				// Operateur
        }
        read_RSSI();														// info RSSI seront ajoutées à message
        uint16_t vbat;
        uint16_t vpct;
        flushSerial();
        fona.getBattVoltage(&vbat);
        fona.getBattPercent(&vpct);
        message += F("Vbat = ");
        message	+= String(vbat);
        message += F(" mV, ");
        message += String(vpct) + "%" + fl;
        if (config.Intru) {
          message += F("Alarme Intrusion ON");
          if (config.IntruAuto) message += F("/Auto");
          message += fl;
        }
        else {
          message += F("Alarme Intrusion OFF");
          if (config.IntruAuto) message += F("/Auto");
          message += fl;
        }
        if (config.Bar) {
          message += F("Alarme Barriere ON");
          message += fl;
        }
        else {
          message += F("Alarme Barriere OFF");
          message += fl;
        }
        message += F("freeRAM=");
        message += String(freeRam()) + fl ;
        message += F("Ver:");
        message += ver;
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
        String temp = textesms.substring(3);
        if (temp.length() > 0 && temp.length() < 11) {
          Id = "";
          temp.toCharArray(config.Idchar, 11);
          sauvConfig();															// sauvegarde en EEPROM
          Id = String(config.Idchar);
          Id += fl;
          sensAlarmeSecteur();
        }
        messageId();
        message += F("Nouvel Id");
        sendSMSReply(callerIDbuffer, sms);
      }

      else if (textesms.indexOf(F("INTRU")) == 0 ) {		//	Alarme Intrusion
        if (textesms.indexOf(F("ON")) == 5) {
          if (!config.Intru) {
            config.Intru = !config.Intru;
            config.IntruAuto = false;
            sauvConfig();															// sauvegarde en EEPROM
            attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
          }
        }
        if (textesms.indexOf(F("OFF")) == 5) {
          if (config.Intru || config.IntruAuto) {
            config.Intru 		 = false;
            config.IntruAuto = false;
            sauvConfig();															// sauvegarde en EEPROM
            detachInterrupt(digitalPinToInterrupt(Ip_PIR));
            /*	Arret Sonnerie au cas ou? sans envoyer SMS */
            digitalWrite(S_Son, LOW);	// Arret Sonnerie
            Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
            FirstSonn = false;
            FlagAlarmeIntrusion = false;
          }
        }
        if (textesms.indexOf(F("AUTO")) == 5) {
          if (!config.IntruAuto) {
            config.IntruAuto = true;
            AIntru_HeureActuelle(); // armement selon l'heure
            // if(config.Intru){
            // attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
            // }
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (config.Intru) {
          message += F("Alarme Intrusion ON");	 //Alarme Intrusion ON
        }
        else {
          message += F("Alarme Intrusion OFF"); //Alarme Intrusion OFF
        }
        if (config.IntruAuto) {
          message += F("/Auto");	 //Alarme Intrusion Auto
        }
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("HINTRU")) == 0 ) {		//	Heures Alarme Intrusion
        if (textesms.indexOf(char(61)) == 6) {	//	"=" changement heure Intru Auto
          // Hintru=Hsoir,Hmatin; Hintru=75600,21600
          int  x = textesms.indexOf(",");
          long i = atol(textesms.substring(7, x).c_str());	// valeur Soir
          long j = atol(textesms.substring(x + 1).c_str());		// valeur Matin
          if (i > 0 && i <= 86340 &&
              j > 0 && j <= 86340); { //	ok si i entre 0 et 86340(23h59) et > Heure matin 	ok si j entre 0 et 86340(23h59) et < Heure soir
            if (config.IntruDebut != i || config.IntruFin != j) { // si changement
              config.IntruDebut 	= i;
              config.IntruFin = j;

              sauvConfig();							// sauvegarde en EEPROM
              Alarm.disable(HIntruD);		//	on arrete les alarmes
              Alarm.disable(HIntruF);
              HIntruF = Alarm.alarmRepeat(config.IntruFin, IntruF);// on parametre
              HIntruD = Alarm.alarmRepeat(config.IntruDebut , IntruD);
              Alarm.enable(HIntruD);		// on redemarre les alarmes
              Alarm.enable(HIntruF);
              AIntru_HeureActuelle();
            }
          }
        }
        message += F("Alarme Intru Auto");
        message += fl;
        message += F("debut : ");
        message += int(config.IntruDebut / 3600);
        message += ":";
        message += int((config.IntruDebut % 3600) / 60);
        message += F("(hh:mm)");
        message += fl;
        message += F("fin      : ");
        message += int(config.IntruFin / 3600);
        message += ":";
        message += int((config.IntruFin % 3600) / 60);
        message += F("(hh:mm)");

        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("CALIBRATION=")) == 0) {
        /* 	Mode calibration mesure tension V2-11
        		Seulement en mode serie local
        		recoit message "CALIBRATION=0"
        		entrer mode calibration
        		effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
        		recoit message "CALIBRATION=1250,825" mesure réelle en V*100
        		V2-11ter 2 valeurs en retour si 2 batterie
        		calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
        		applique nouveau coeff
        		stock en EEPROM
        		sort du mode calibration

        		variables
        		FlagCalibration true cal en cours, false par defaut
        		static int tensionmemo memorisation de la premiere tension mesurée en calibration
        		int CoeffTension = CoeffTensionDefaut 3100 par défaut
        */
        String bidon = textesms.substring(12, 16);
        bool valide = false;
        //Serial.print(F("bidon=")),Serial.print(bidon),Serial.print(","),Serial.println(bidon.length());
        if (bidon.substring(0, 1) == "0" ) { // debut mode cal
          FlagCalibration = true;
          CoeffTension = CoeffTensionDefaut;
          VBatterie1 = map(moyenneAnalogique(Ip_AnalBatt1), 0, 1023, 0, CoeffTension);
          // Serial.print("VBatterie1 = "),Serial.println(VBatterie1);
          tensionmemo1 = VBatterie1;
          if (config.Batterie2) {	// V2-11ter
            CoeffTension2 = CoeffTensionDefaut;
            VBatterie2 = map(moyenneAnalogique(Ip_AnalBatt2), 0, 1023, 0, CoeffTension2);
            // Serial.print("VBatterie1 = "),Serial.println(VBatterie1);
            tensionmemo2 = VBatterie2;
          }
        }
        else if (FlagCalibration && bidon.substring(0, 4).toInt() > 0 && bidon.substring(0, 4).toInt() <= 3300) {
          // si Calibration en cours et valeur entre 0 et 5000

          /* calcul nouveau coeff */
          CoeffTension = bidon.substring(0, 4).toFloat() / float(tensionmemo1) * CoeffTensionDefaut;
          // Serial.print("Coeff Tension = "),Serial.println(CoeffTension);
          VBatterie1 = map(moyenneAnalogique(Ip_AnalBatt1), 0, 1023, 0, CoeffTension);
          valide = true;
          // Serial.print("VBatterie1 = "),Serial.println(VBatterie1);
          if (config.Batterie2) { // V2-11ter
            bidon = textesms.substring(17, 21);
            if (bidon.substring(0, 4).toInt() > 0 && bidon.substring(0, 4).toInt() <= 3300) {
              CoeffTension2 = bidon.substring(0, 4).toFloat() / float(tensionmemo2) * CoeffTensionDefaut;
              VBatterie2 = map(moyenneAnalogique(Ip_AnalBatt2), 0, 1023, 0, CoeffTension2);
              valide = true;
            }
            else {
              valide = false;
            }
          }
          FlagCalibration = false;
          if (valide){															// sauvegarde en EEPROM
            EEPROM_writeAnything(EEPROM_adresse[1], CoeffTension);
            Alarm.delay(100);
            EEPROM_writeAnything(EEPROM_adresse[2], CoeffTension2);
            Alarm.delay(100);
          }
        }
        message += F("Mode Calib Tension");
        message += fl;
        message += F("VBatterie1 = ");
        message += VBatterie1;
        message += fl;
        message += F("Coeff Tension = ");
        message += CoeffTension;
        message += fl;
        message += F("VBatterie2 = ");
        message += VBatterie2;
        message += fl;
        message += F("Coeff Tension = ");
        message += CoeffTension2;
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("BAR")) == 0 ) {		//	Alarme position Barriere
        if (textesms.indexOf(F("ON")) == 3) {
          if (!config.Bar) {
            config.Bar = !config.Bar;
            sauvConfig();															//	sauvegarde en EEPROM
          }
        }
        if (textesms.indexOf(F("OFF")) == 3) {
          if (config.Bar) {
            config.Bar = !config.Bar;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (config.Bar) {
          message += F("Alarme Barriere ON");					//	Alarme Barriere ON
        }
        else {
          message += F("Alarme Barriere OFF"); 				//	Alarme Barriere OFF
        }
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("SILENCE")) == 0 ) {		//	Alarme Silencieuse
        if (textesms.indexOf(F("ON")) == 7) { //ON
          if (!config.Silence) {
            config.Silence = !config.Silence;
            //	V2-11bis
            /*	Arret Sonnerie au cas ou? sans envoyer SMS */
            digitalWrite(S_Son, LOW);	// Arret Sonnerie
            //	V2-11bis
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (textesms.indexOf(F("OFF")) == 7) {
          if (config.Silence) {
            config.Silence = !config.Silence;
            sauvConfig();															// sauvegarde en EEPROM
            //	V2-11bis
            /*	Arret Sonnerie au cas ou? sans envoyer SMS */
            // digitalWrite(S_Son, LOW);	// Arret Sonnerie
            // Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            // Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
            //	V2-11bis
          }
        }
        generationMessage();
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("MODETEST")) == 0 ) {		//	mode test reduction tempo Sonnerie
        if ((textesms.indexOf("?") == 8) || (textesms.indexOf(char(61)) == 8)) { //char(61) "="
          if (textesms.indexOf(F("ON")) == 9) {						//ON
            ModeTest = true;
            Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            TSonn = Alarm.timerRepeat(1, ArretSonnerie);	// tempo sonnerie 1s
            //Alarm.enable(TSonn);			// on arrete la tempo sonnerie
            CptTest = 12;
          }
          if (textesms.indexOf(F("OFF")) == 9) {
            ModeTest = false;
            Alarm.disable(TSonn);			// on arrete la tempo sonnerie
            TSonn = Alarm.timerRepeat(config.Dsonn, ArretSonnerie);		// tempo sonnerie	normale
            //Alarm.enable(TSonn);
            CptTest = 0;
          }
          if (ModeTest) {
            message += F("Mode test ON");								//Mode test ON
          }
          else {
            message += F("Mode test OFF"); 							//Mode test OFF
          }
          sendSMSReply(callerIDbuffer, sms);
        }
      }

      else if (textesms.indexOf(F("POSPN")) == 0 ) {		//	Info PN Fermé
        if (textesms.indexOf(F("ON")) == 5) {						//ON
          if (!config.Pos_PN) {
            config.Pos_PN = !config.Pos_PN;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (textesms.indexOf(F("OFF")) == 5) {
          if (config.Pos_PN) {
            config.Pos_PN = !config.Pos_PN;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        if (config.Pos_PN) {
          message += F("Info position PN ON");	// Info position PN ON
        }
        else {
          message += F("Info position PN OFF"); // Info position PN OFF
        }
        sendSMSReply(callerIDbuffer, sms);
      }

      else if (textesms.indexOf(F("LSTPOSPN")) == 0 ) {		//	Liste restreinte Info PN Fermé //  =LSTPOSPN=1,0,0,0,0,0,0,0,1,
        if ((textesms.indexOf("?") == 8) || (textesms.indexOf(char(61)) == 8)) { //char(61) "="
          byte Num[10];
          String bidon = textesms.substring(9, 27);
          Serial.println(F("bidon=")), Serial.print(bidon), Serial.println(bidon.length());
          if (bidon.length() == 18) { //18
            //bool Num[10];
            int j = 1;
            for (int i = 0; i < 18; i += 2) {		//18
              if ((bidon.substring(i + 1, i + 2) == ",") && (bidon.substring(i, i + 1) == "0"	|| bidon.substring(i, i + 1) == "1")) {
                //Serial.print(",="),Serial.println(bidon.substring(i+1,i+2));
                //Serial.print("X="),Serial.println(bidon.substring(i,i+1));
                Num[j] = bidon.substring(i, i + 1).toInt();
                //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
                j++;
              }
              else {
                Serial.println(F("pas bon"));
                goto FinLSTPOSPN;	// pas bon on sort
              }
            }
            //Serial.println("copie des num");
            for (int i = 1; i < 10; i++) {
              config.Pos_Pn_PB[i] = Num[i];
            }
            sauvConfig();															// sauvegarde en EEPROM
          }
FinLSTPOSPN:
          message += F("Liste Position PN");
          message += fl;
          for (int i = 1; i < 10; i++) {
            message += config.Pos_Pn_PB[i];
            message += char(44);						// ,
          }
          sendSMSReply(callerIDbuffer, sms);
        }
      }
      else if (textesms.indexOf(F("VIE")) == 0) {			//	Heure Message Vie
        if ((textesms.indexOf(char(61))) == 3) {
          long i = atol(textesms.substring(4).c_str()); //	Heure message Vie
          if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
            config.Ala_Vie = i;
            sauvConfig();														// sauvegarde en EEPROM
            Alarm.disable(Svie);
            Svie = Alarm.alarmRepeat(config.Ala_Vie, SignalVie);	// init tempo
            Alarm.enable(Svie);
          }
        }
        message += F("Heure Vie = ");
        message += int(config.Ala_Vie / 3600);
        message += ":";
        message += int((config.Ala_Vie % 3600) / 60);
        message += F("(hh:mm)");
        //message += fl;
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("PARAM")) == 0) {				//	Parametres fausses alarmes
        if (textesms.indexOf(char(61)) == 5) {
          int x = textesms.indexOf(":");
          int i = atoi(textesms.substring(6, x).c_str());	//	nombre de fausses Alarmes
          int j = atoi(textesms.substring(x + 1).c_str());// duree analyse
          Serial.print(i), Serial.print(","), Serial.println(j);
          if (i > -1 && i < 101 && j > 9 && j < 601) {
            // nombre entre 1 et 100, durée entre 10 et 600
            config.Nmax = i;
            config.timecomptemax = j * 10; //passage en n*100ms
            sauvConfig();																// sauvegarde en EEPROM
          }
        }
        message += F("Parametres Fausses Alarmes");
        message += fl;
        message += F("n = ");
        message += config.Nmax;
        message += F(", t = ");
        message += config.timecomptemax / 10;
        message += F("(s)");
        sendSMSReply(callerIDbuffer, sms);
      }
			else if (textesms.indexOf(F("FALARME")) == 0){ // fausses alarmes V2-12
				MessageFaussesAlarmes(false);
				sendSMSReply(callerIDbuffer, sms);				
			}
      else if (textesms.indexOf(F("SONN")) == 0) {			//	Durée Sonnerie  V2-11
        if ((textesms.indexOf(char(61))) == 4) {
          int x = textesms.indexOf(":");
          int y = textesms.indexOf(":", x + 1);

          long i = atol(textesms.substring(5, x).c_str()); 	//	Dsonn Sonnerie
          long j = atol(textesms.substring(x + 1, y).c_str()); //	DsonnMax Sonnerie
          long k = atol(textesms.substring(y + 1).c_str()); 	//	Dsonnrepos Sonnerie
          //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(k);
          if (i > 5  && i <= 300 &&
              j > i  && j <= 600 &&
              k > 10 && k <= 300) {			//	ok si entre 10 et 300
            config.Dsonn 			= i;
            config.DsonnMax 	= j;
            config.Dsonnrepos = k;
            sauvConfig();																// sauvegarde en EEPROM
          }
        }
        message += F("Param Sonnerie = ");
        message += config.Dsonn;
        message += ":";
        message += config.DsonnMax;
        message += ":";
        message += config.Dsonnrepos;
        message += "(s)";
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("SIRENE")) == 0) { // Lancement SIRENE V2-11
        ActivationSonnerie();
        message += F("Lancement Sonnerie");
        message += fl;
        message += config.Dsonn;
        message += F("(s)");
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("TIME")) == 0) {		//	Heure Systeme
        message += F("Heure Sys = ");
        displayTime(true);
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure V2-11ter
        if(sms){
          char datebuffer[21];
          fona.getSMSdate(slot, datebuffer, 20);
          String mytime = String(datebuffer).substring(0,20);
          // Serial.print(F("heure du sms:")),Serial.println(mytime);
          String _temp = F("AT+CCLK=\"");
          _temp += mytime + "\"\r\n";
          // Serial.print(_temp);
          fona.print(_temp);// mise a l'heure SIM800
          Alarm.delay(100);
          MajHeure();			// mise a l'heure
          }
        else{
          message += F("pas de mise à l'heure en local");
        }
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("BATTERIE2")) == 0) {	// V2-11ter
        if ((textesms.indexOf(char(61))) == 9) {
          if (textesms.indexOf(F("ON")) == 10) {
            if (!config.Batterie2) {
              config.Batterie2 = true;
              sauvConfig();
            }
          }
          if (textesms.indexOf(F("OFF")) == 10) {
            if (config.Batterie2) {
              config.Batterie2 = false;
              sauvConfig();
            }
          }
        }
        message += F("Batterie2 ");
        if (config.Batterie2) {
          message += F("ON");
        }
        else {
          message += F("OFF");
          message += fl;
        }
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms == F("RST")) {								// demande RESET
        message += F("Le systeme va etre relance");		// apres envoie du SMS!
        FlagReset = true;															// reset prochaine boucle
        sendSMSReply(callerIDbuffer, sms);
      }
      else if (textesms.indexOf(F("IMEI")) == 0) {			// V2-11bis recuperer IMEI
        char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
        uint8_t imeiLen = fona.getIMEI(imei);
        if (imeiLen > 0) {
          Serial.print(F("Module IMEI: ")), Serial.println(imei);
          message += F("IMEI = ");
          message += String(imei);
          sendSMSReply(callerIDbuffer, sms);
        }
      }
      else {
        message += F("Commande non reconnue ?");		//"Commande non reconnue ?"
        sendSMSReply(callerIDbuffer, sms);
      }
      //Serial.print(message);
    }
    else {
      Serial.print(F("Appelant non reconnu ! ")), Serial.println(callerIDbuffer);
    }
sortir:
    Serial.println("")		; /* test seulement */
    // Suppression du SMS
    flushSerial();
    if (sms) {
      if (fona.deleteSMS(slot)) {
        Serial.print(F("OK! message supprime, slot=")), Serial.println(slot);
      } else {
        Serial.print(F("Impossible de supprimer slot=")), Serial.println(slot);
      }
    }
  }
}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;
  byte groupe = 0;

  if (FlagAlarmeSect != FlagLastAlarmeSect) { // si nouvelle alarme on envoie Etat
    SendEtat = true;
    FlagLastAlarmeSect = FlagAlarmeSect;
  }
  if (FlagAlarmeBatt != FlagLastAlarmeBatt) {
    SendEtat = true;
    FlagLastAlarmeBatt = FlagAlarmeBatt;
  }
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (FlagAlarmeIntrusion != FlagLastAlarmeIntrusion) {
    SendEtat = true;
    FlagLastAlarmeIntrusion = FlagAlarmeIntrusion;
  }
  if (FlagAlarmeBar != FlagLastAlarmeBar) {
    SendEtat = true;
    FlagLastAlarmeBar = FlagAlarmeBar;
  }
  if (FlagPNFerme != FlagLastPNFerme) {
    if (config.Pos_PN || Nuit()) {		// si demande SMS Position PN ou Nuit
      SendEtat = true;
      groupe = 1;											// liste restreinte
    }
    FlagLastPNFerme = FlagPNFerme;
  }
  if (SendEtat) { 								// si envoie Etat demandé
    envoieGroupeSMS(groupe);			// envoie groupé V1-2
    SendEtat = false;							// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp) {
  /* si grp = 0,
  	envoie un SMS à tous les numero existant (9 max) du Phone Book
  	si grp = 1,
  	envoie un SMS à tous les numero existant (9 max) du Phone Book
  	de la liste restreinte config.Pos_Pn_PB[x]=1			*/
  Serial.print(F("Sms groupe = ")), Serial.println(grp);
  for (byte Index = 1; Index < 10; Index++) {		// Balayage des Num Tel Autorisés=dans Phone Book
    //wdt_reset();
    if (!fona.getPhoneBookNumber(Index, Telbuff, 13)) { // lire Phone Book
      Serial.print(Index), Serial.println(F("Failed!"));
      break;
    }
    Serial.print(F("Num :  ")), Serial.println(Telbuff);
    if (String(Telbuff).length() > 0)	{	// Numero Tel existant/non vide
      if (grp == 1) {	// grp = 1 message liste restreinte
        if (config.Pos_Pn_PB[Index] == 1) {
          generationMessage();
          // message += F("special");
          sendSMSReply(Telbuff, true);
        }
      }
      else {	// grp = 0, message à tous
        generationMessage();
        sendSMSReply(Telbuff, true);
      }
    }
  }
}
//---------------------------------------------------------------------------
void generationMessage() {
  /* Generation du message etat/alarme général */

  messageId();
  if (FlagAlarmeBatt || FlagAlarmeSect || FlagAlarmeTension
      || FlagLastAlarmeBatt || FlagLastAlarmeSect
      || FlagLastAlarmeTension || FlagAlarmeIntrusion
      || FlagAlarmeBar) {
    message += F("--KO--------KO--");//+= V1-21
  }
  else {
    message += F("-------OK-------");//+= V1-21
  }
  message += fl;

  message += F("Secteur : ");				//"Alarme Secteur : "
  if (FlagAlarmeSect || FlagLastAlarmeSect) {
    message += F("Alarme");
    message += fl;
  }
  else {
    message += F("OK");
    message += fl;
  }
  message += F("Batterie : ");				//"Alarme Batterie : "
  if (FlagAlarmeBatt || FlagLastAlarmeBatt || FlagAlarmeTension) {
    message += F("Alarme");
    message += fl;
  }
  else {
    message += F("OK");
    message += fl;
  }
  message += F("VBatterie = ");			//"Tension Batterie = " V2-11ter
  message += String(VBatterie1 / 100) + ",";							//V1.1
  if ((VBatterie1 - (VBatterie1 / 100) * 100) < 10) { //correction bug decimal<10
    message += "0";
  }
  message += VBatterie1 - ((VBatterie1 / 100) * 100);	//V1.1
  message += "V";
  
  if (!config.Batterie2) {															//V2-11ter
    message += fl;
  } else {
    message += ";";
    message += String(VBatterie2 / 100) + ",";
    if ((VBatterie2 - (VBatterie2 / 100) * 100) < 10) {
      message += "0";
    }
    message += VBatterie2 - ((VBatterie2 / 100) * 100);
    message += "V";
  }																										//V2-11ter
  if(FlagAlarmeTension){ // V2-14
    message += F(" KO");
  } else{
    message += F(" OK");
  }
  message += fl;
  if (FlagPNFerme) {													// PN fermé
    message += F("PN Ferme");
    message += fl;
  } else {
    if(dureeFermeture > 0){
      message += F("Fermeture : ");
      message += dureeFermeture;
      message += "s" + fl;
      dureeFermeture = 0;
    }
  }
  if (config.Bar && FlagAlarmeBar) {					// Barriere tombée
    message += F("Alarme Barriere");
    message += fl;
    message += F("Br1 : ");
    if (Br1) {
      message += F("Tombee");
    }
    else {
      message += F("OK");
    }
    message += fl;

    message += F("Br2 : ");
    if (Br2) {
      message += F("Tombee");
    }
    else {
      message += F("OK");
    }
    message += fl;
  }
  if (config.Intru) {
    if(FlagAlarmeIntrusion){
      message += fl ;
      message += F("-- Intrusion !--") ;			// Intrusion !
      message += fl ;
      message += F("Alarme = ");
      message += FausseAlarme;
      message += fl ;
    }
  }
    if (config.Silence) {
      message += F("Silence ON");
      message += fl;
    }
    else
    {
      message += F("Silence OFF");
      message += fl;
    }
}

//---------------------------------------------------------------------------
void sendSMSReply(char *num, bool sms) { //char *cmd  String message
  // si sms=true Envoie SMS, sinon Serialprint seulement
  if (sms) {
    //wdt_reset();
    if (!fona.sendSMS(num, message.c_str())) {
      Serial.println(F("Envoi SMS Failed"));
    } else {
      Serial.println(F("SMS Sent OK"));
    }
  }
  Serial.print (F("Message (long) = ")), Serial.println(message.length());
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  int r;
  byte n = fona.getRSSI();
  // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  message += F("RSSI=");
  message += String(n);
  message += ", ";
  message += String(r);
  message += F("dBm");
  message += fl;
}
//---------------------------------------------------------------------------
void VerifBarriere() {
  /* 	Lecture position des barrieres true=fermée
  	si pas de tension sur DFV et CFV
  	apres fermeture commandé DFV et CFV sous tension
  	on attend 60s avant d'autoriser les mesures
  	temps necessaire à la remontée des barrieres */
  static bool lastFlagDFV = false;
  static bool lastFlagCFV = false;
  FlagDFV = !digitalRead(DFV);// true = pas de mesure position barrieres possible
  FlagCFV = !digitalRead(CFV);// true = pas de mesure position barrieres possible
  if(FlagDFV && FlagDFV != lastFlagDFV){ // début Fermeture
    lastFlagDFV = FlagDFV;
    timedebutFermeture = now();
    dureeFermeture = 0;
  } else if(FlagCFV && FlagCFV != lastFlagCFV){ // Fermé
    lastFlagCFV = FlagCFV;
    FlagPNFerme = true;
  } else if(!FlagDFV && FlagDFV != lastFlagDFV && !FlagCFV && FlagCFV != lastFlagCFV){// Ouvert
    FlagPNFerme = false;
    lastFlagDFV = FlagDFV;
    lastFlagCFV = FlagCFV;
    timefinFermeture = now();
    dureeFermeture = timefinFermeture - timedebutFermeture;
  }

  Serial.print(F("Flag DFV, last = ")), Serial.print(FlagDFV),Serial.print(','),Serial.print(lastFlagDFV);
  Serial.print(F(", Flag CFV, last = ")), Serial.print(FlagCFV),Serial.print(','),Serial.print(lastFlagCFV);
  Serial.print(F(", temps fermeture = ")), Serial.print(now() - timedebutFermeture);
  Serial.print(", garde:"),Serial.println((now() - timefinFermeture));

  if (!FlagDFV && !FlagCFV) {	// seulement si pas de tension sur le circuit de mesure
    if (now() - timefinFermeture > 60) { // on attend 60s, apres ouverture, avant autorisation lecture barriere
      digitalWrite(S4, HIGH);	// on ferme tous les optos
      digitalWrite(S3, HIGH);
      digitalWrite(S2, HIGH);
      digitalWrite(S1, HIGH);
      delay(200);
      Br1 = digitalRead(E1);		// on lit l'etat des barrieres
      Br2 = digitalRead(E2);
      digitalWrite(S1, LOW);		// on ouvre tous les optos
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      digitalWrite(S4, LOW);

      if (!Br1) {
        Serial.print (F("Barriere 1 = Ouverte, "));
      }
      else {
        Serial.print (F("Barriere 1 = Ferme, "));
      }
      if (!Br2) {
        Serial.println (F("Barriere 2 = Ouverte"));
      }
      else {
        Serial.println (F("Barriere 2 = Ferme"));
      }

      if (Br1 || Br2) {	//	Barriere fermée
        FlagAlarmeBar = true;
      }
      else {	// les 2 barrieres ouvertes
        FlagAlarmeBar = false;
        lastFlagDFV = FlagDFV;
        lastFlagCFV = FlagCFV;
      }
    }
  }
  else {						// si presence tension, le PN est fermé ou en cours de fermeture
    FlagAlarmeBar = false;
    Serial.println(F("pas de mesure barrieres"));
  }

}
//---------------------------------------------------------------------------
void MajHeure() {
  /* module identique toutes version
  	procedure appelée toute les heures
  	parametrage du SIM800 a faire une fois
  	AT+CLTS? si retourne 0
  	AT+CLTS=1
  	AT+CENG=3
  	AT&W pour sauvegarder ce parametre
  	si AT+CCLK? pas OK
  	avec Fonatest passer en GPRS 'G', envoyer 'Y' la sync doit se faire, couper GPRS 'g'
  	't' ou AT+CCLK? doit donner la date et heure réseau
  	format date retourné par Fona "yy/MM/dd,hh:mm:ss±zz",
  	+CCLK: "14/08/08,02:25:43-16" -16= décalage GMT en n*1/4heures(-4) */
  static bool First = true;
  //Serial.print(F("getTriggeredAlarmId()MajH="));
  //Serial.println(Alarm.getTriggeredAlarmId());
  int N_Y;	// variable tempo  New_Year
  int N_M;
  int N_D;
  int N_H;
  int N_m;
  int N_S;

  char buffer[23];
  fona.getTime(buffer, 23);  // demande heure réseau : AT+CCLK?
  String FonaHDtate = buffer;
  //if(First) FonaHDtate = "14/10/20,11:20:00-0   ";// pour test
  //Serial.print("buffer="),Serial.println(buffer);
  //Serial.print("String="),Serial.println(FonaHDtate);

  // convertir format date time yy/mm/dd,hh:mm:ss
  byte i 	= FonaHDtate.indexOf("/");
  byte j 	= FonaHDtate.indexOf("/", i + 1);
  N_Y			= FonaHDtate.substring(i - 2, i).toInt();
  N_M 		= FonaHDtate.substring(i + 1, j).toInt();
  N_D 		= FonaHDtate.substring(j + 1, j + 3).toInt();
  i 	  	= FonaHDtate.indexOf(":", 6);
  j     	= FonaHDtate.indexOf(":", i + 1);
  N_H 		= FonaHDtate.substring(i - 2, i).toInt();
  N_m 		= FonaHDtate.substring(i + 1, j).toInt();
  N_S 		= FonaHDtate.substring(j + 1, j + 3).toInt();

  //Serial.print(N_H),Serial.print(":"),Serial.print(N_m),Serial.print(":"),Serial.print(N_S),Serial.print(" ");
  //Serial.print(N_D),Serial.print("/"),Serial.print(N_M),Serial.print("/"),Serial.println(N_Y);
  Serial.print(F("Mise a l'heure reguliere !, "));

  if (First) {																// premiere fois apres le lancement
    setTime(N_H, N_m, N_S, N_D, N_M, N_Y);	// mise à l'heure de l'Arduino
    First = false;
  }
  else {
    //  calcul décalage entre H sys et H reseau en s
    int ecart = (N_H - hour()) * 3600;
    ecart += (N_m - minute()) * 60;
    ecart += N_S - second();
    Serial.print(F("Ecart s= ")), Serial.println(ecart);

    if (abs(ecart) > 5) {
      Alarm.disable(loopPrincipale);
      // V2-11bis
      ArretSonnerie();	// Arret Sonnerie propre correction bug blocagealarme
      // V2-11bis
      Alarm.disable(TSonn);						// les tempos sonnerie sont coupées au cas ou active à ce moment là
      Alarm.disable(TSonnMax);				// mais elles ne sont pas réarmées, elles le seront si nouvelles alarme
      Alarm.disable(TSonnRepos);
      Alarm.disable(MajH);//v1-15
      Alarm.disable(Svie);
      Alarm.disable(HIntruD);
      Alarm.disable(HIntruF);
      setTime(N_H, N_m, N_S, N_D, N_M, N_Y);// V1-15
      //V1-15 adjustTime(ecart);	// correction heure interne Arduino
      Alarm.enable(loopPrincipale);
      Alarm.enable(MajH);//v1-15
      Alarm.enable(Svie);
      Alarm.enable(HIntruD);
      Alarm.enable(HIntruF);

      //Serial.print(F("Correction seconde = ")), Serial.println(ecart);
    }
  }
  displayTime(false);
  timesstatus();
  MessageFaussesAlarmes(true);
  AIntru_HeureActuelle(); // armement selon l'heure
}
//---------------------------------------------------------------------------
void MessageFaussesAlarmes(bool sms) {
  // sms = true envoie du sms et RAZ V2-12
	// sms = false creation du message sans RAZ, sms sera envoyé par procedure appelante
	messageId();
  if (FausseAlarme > 0 ) {	// Si fausse alarme envoie sms info nbr fausse alarme
    Serial.print(F("MAJH, Nombre fausse alarmes : "));
    Serial.println(FausseAlarme);

    message += F("Fausses Alarmes : ");
    message += FausseAlarme;
		if(sms){
			byte Index = 1;										// message au 1er tel
			fona.getPhoneBookNumber(Index, Telbuff, 13);
			sendSMSReply(Telbuff, true);
			Index = 2;												// message au 2eme tel
			if (fona.getPhoneBookNumber(Index, Telbuff, 13)) { // lire Phone Book si Index present
				fona.getPhoneBookNumber(Index, Telbuff, 13);
				sendSMSReply(Telbuff, true);
			}
			FausseAlarme = 0;
		}
  }
  else{
		message += F("Pas de fausses Alarmes");
	}
}
//---------------------------------------------------------------------------
void ActivationSonnerie() {
  // Sonnerie PN
  if (!SonnMax) {									// pas atteint Temps sonnerie maxi
    if (!config.Silence) digitalWrite(S_Son, HIGH);		// Marche Sonnerie sauf Silence
    Alarm.enable(TSonn);
    if (!FirstSonn) {							// premiere sonnerie on lance Tempo Max
      Alarm.enable(TSonnMax);
      FirstSonn = true;
    }
  }
}
//---------------------------------------------------------------------------
void ArretSonnerie() {
  Serial.print(F("Fin tempo Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  digitalWrite(S_Son, LOW);	// Arret Sonnerie
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  FirstSonn = false;
  FlagAlarmeIntrusion = false;
  FlagPIR = false;
}
//---------------------------------------------------------------------------
void SonnerieMax() {
  Serial.print(F("Fin periode Max Sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo temps de sonnerie maxi
  Alarm.enable(TSonnRepos);	// on lance la tempo repos sonnerie
  Alarm.disable(TSonnMax);	// on arrete la tempo sonnerie maxi
  Alarm.disable(TSonn);			// on arrete la tempo sonnerie
  digitalWrite(S_Son, LOW);	// Arret Sonnerie
  FirstSonn = false; ///
  SonnMax = true;							// interdit nouveau lancement sonnerie
}
//---------------------------------------------------------------------------
void ResetSonnerie() {
  Serial.print(F("Fin periode inhibition sonnerie : "));
  Serial.println(Alarm.getTriggeredAlarmId());
  // fin de la tempo repos apres temps sonnerie maxi
  Alarm.disable(TSonnRepos);// on arrete la tempo repos
  SonnMax = false;
  ArretSonnerie();
}
//---------------------------------------------------------------------------
void OnceOnly() {
  Serial.println(F("Premier message apres lancement !"));
  Serial.print(F("getTriggeredAlarmId() Once="));
  Serial.println(Alarm.getTriggeredAlarmId());
  if (FlagTempoIntru) {			//	si Alarme Intru était demandée
    config.Intru = true;		//	on reactive
    attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
  }

  envoieGroupeSMS(1);	//  envoie message etat apres lancement à liste pref
}

//---------------------------------------------------------------------------
void SignalVie() {
  Serial.print(F("boucle SignalVie "));
  displayTime(false);
  // ************************ boucle signe de vie ***************************
  Ntwk_dcx = 0;					// reset compteur deconnexion reseau
  envoieGroupeSMS(0);// envoie groupé V1-2
  fona.println(F("AT+CMGDA=\"DEL ALL\""));// au cas ou, efface tous les SMS envoyé/reçu

}
//---------------------------------------------------------------------------
void sauvConfig() {
  // Sauvegarde config en EEPROM
  Serial.println(F("Enregistrement EEPROM"));
  byte n = EEPROM_writeAnything(EEPROM_adresse[0], config);		//	ecriture EEPROM
  Alarm.delay(5);
  EEPROM_readAnything(EEPROM_adresse[0], config);
  Alarm.delay(500);	//	V2-12
}
//---------------------------------------------------------------------------
void displayTime(bool m) {
  // m = true ajouter Time à message
  String dt;
  if (day() < 10) {
    dt += "0";
  }
  dt += day();
  dt += ("/");
  if (month() < 10) {
    dt += "0";
  }
  dt += month();
  dt += ("/");
  dt += year();
  dt += (" ");
  if (hour() < 10) {
    dt += "0";
  }
  dt += hour();
  dt += ":";
  if (minute() < 10) {
    dt += "0";
  }
  dt += minute();
  dt += ":";
  if (second() < 10) {
    dt += "0";
  }
  dt += second();
  if (m) message += dt;
  Serial.println(dt);
}
//---------------------------------------------------------------------------
void flushSerial() {
  while (Serial.available())
    Serial.read();
}
//---------------------------------------------------------------------------
void timesstatus() {	// etat synchronisation time/heure systeme
  Serial.print(F("Synchro Time  : "));
  switch (timeStatus()) {
    case 0:
      Serial.println(F(" pas synchro"));
      break;
    case 1:
      Serial.println(F(" defaut synchro"));
      break;
    case 2:
      Serial.println(F(" OK"));
      break;
  }
}
//---------------------------------------------------------------------------
void softReset() {
  //asm volatile ("  jmp 0");	//	Reset Soft
  wdt_enable(WDTO_250MS);					// activation du watchdog

  while (1);

}
//---------------------------------------------------------------------------
int freeRam () {	// lecture RAM free
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
//---------------------------------------------------------------------------
bool HeureEte() {
  // return true en été, false en hiver (1=dimanche)
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  return Hete;
}
//---------------------------------------------------------------------------
void IntruF() { // Parametrage Alarme Intrusion
  Serial.println(F("Fin periode intru"));
  if (config.IntruAuto) {
    config.Intru = false;	//  Alarme OFF
    detachInterrupt(digitalPinToInterrupt(Ip_PIR));
    Serial.print(F("Intru = ")), Serial.println(config.Intru);
  }
}
//---------------------------------------------------------------------------
void IntruD() { // Parametrage Alarme Intrusion
  Serial.println(F("Debut periode intru"));
  if (config.IntruAuto) {
    config.Intru = true;	//  Alarme ON
    attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
    CptAlarme 	 = 0;
    FausseAlarme = 0;
    Serial.print(F("Intru = ")), Serial.println(config.Intru);
  }
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() {
  // armement Alarme Intru en fonction de l'heure actuelle
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes

  if (config.IntruAuto) {
    if (Nuit()) {
      config.Intru = true;
      attachInterrupt(digitalPinToInterrupt(Ip_PIR), IRQ_PIR, RISING);
    }
    else {
      config.Intru = false;
      detachInterrupt(digitalPinToInterrupt(Ip_PIR));
    }
  }
  Serial.print(F("Hintru = ")), Serial.print(Heureactuelle), Serial.print(",");
  Serial.print(config.IntruDebut), Serial.print(","), Serial.print(config.IntruFin);
  Serial.print(","), Serial.println(config.Intru);
}
//--------------------------------------------------------------------------------//
bool Nuit() {
  bool nuit = false; // periode nuit alarme active
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes

  if (config.IntruDebut > config.IntruFin) {
    if ((Heureactuelle > config.IntruDebut && Heureactuelle > config.IntruFin)
        || (Heureactuelle < config.IntruDebut && Heureactuelle < config.IntruFin)) {
      nuit = true;
    }
    else {
      nuit = false;
    }
  }
  else {
    if (Heureactuelle > config.IntruDebut && Heureactuelle < config.IntruFin) {
      nuit = true;
    }
    else {
      nuit = false;
    }
  }
  return nuit;
}
//--------------------------------------------------------------------------------//
int moyenneAnalogique(byte Input) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    delay(10);
    moyenne += analogRead(Input);
  }
  moyenne /= 10;
  return moyenne;
}
//--------------------------------------------------------------------------------//
void messageId() { // V2-21
  message = Id;
  displayTime(true); // V2-21
  message += fl;     // V2-21
}
//--------------------------------------------------------------------------------//
void ResetSIM800() { // V2-11ter
  fonaSerial -> println(F("AT+CFUN=1,1"));
  Alarm.delay(1000);
  fonaSerial -> println(F("AT+CLTS=1"));
  fonaSerial -> println(F("AT+CENG=3"));
  if (!fona.getetatSIM()) {	// Si carte SIM not READY, Envoyé PIN
    flushSerial();
    char PIN[5] = "1234";
    byte retries = 1;
    if (! fona.unlockSIM(PIN)) {
      Serial.println(F("Failed to unlock SIM"));
      retries++;
      Alarm.delay(1000);
      if (retries == 3) {
        goto sortie;					// 2 tentatives max
      }
    }
    else {
      Serial.println(F("OK SIM Unlock"));
    }
sortie:
    Alarm.delay(1000);				//	Attendre cx reseau apres SIM unlock
  }
}
//--------------------------------------------------------------------------------//
void sensAlarmeSecteur(){
  if(Id.substring(7,9) == "56" || Id.substring(7,9) == "62"){
    TypePN = true;
  } else {TypePN = false; }//PN64
}
/* --------------------  test seulement ----------------------*/
void recvOneChar() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    demande += receivedChar;
    if (receivedChar == 10) {
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.println(demande);
    interpretemessage();
    newData = false;
  }
}
void interpretemessage() {
  String bidons;
  demande.toUpperCase();
  if (demande.indexOf("=") == 0) {
    bidons = demande.substring(1); //(demande.indexOf("=")+1);
    int lon0 = bidons.length();
    bidons.toCharArray(replybuffer, lon0 - 1);	// len-1 pour supprimer lf
    //Serial.print(lon0),Serial.print(","),Serial.print(bidons),Serial.print(","),Serial.println(replybuffer);
    traite_sms(99);	//	traitement SMS en mode test local
  }

  demande = "";
}
