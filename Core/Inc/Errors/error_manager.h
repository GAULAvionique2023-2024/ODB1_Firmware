#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ERR_OK = 0,
    
    // Capteurs (1000-1099)
    ERR_SENSOR1_FAIL      = 1000,
    ERR_SENSOR2_FAIL      = 1001,
    ERR_SENSOR1_RANGE     = 1002,
    ERR_SENSOR2_RANGE     = 1003,
    
    // Communication (1100-1199)
    ERR_COMM1_TIMEOUT     = 1100,
    ERR_COMM2_TIMEOUT     = 1101,
    ERR_COMM_FRAME_ERROR  = 1102,
    
    // Alimentation (1200-1299)
    ERR_POWER_UNDERVOLTAGE = 1200,
    ERR_POWER_OVERVOLTAGE  = 1201,
    
    ERR_CODE_MAX
} ErrorCode;


typedef enum {
    ERR_LEVEL_INFO = 0,      // Information seulement
    ERR_LEVEL_WARNING,       // Attention requise
    ERR_LEVEL_ERROR,         // Erreur mais récupérable
    ERR_LEVEL_CRITICAL,      // Erreur critique, safing requis
    ERR_LEVEL_FATAL          // Erreur fatale, abort mission
} ErrorLevel;


typedef enum {
    SRC_NONE = 0,
    SRC_SENSOR,
    SRC_COMM,
    SRC_THERMAL,
    SRC_POWER,
    SRC_FLIGHT_CONTROL,
    SRC_MAX
} ErrorSource;


typedef enum {
    STATE_INIT,              // Initialisation
    STATE_PREFLIGHT,         // Prévol / attente
    STATE_ARMED,             // Armé, prêt au décollage
    STATE_FLIGHT,            // En vol
    STATE_RECOVERY,          // Récupération (parachute)
    STATE_LANDED,            // Atterri
    STATE_ERROR,             // Erreur modérée
    STATE_CRITICAL,          // Erreur critique
    STATE_SAFE_MODE,         // Mode sûr (safing)
    STATE_DEBUG              // Mode debug
} SystemState;


typedef struct {
    uint16_t    code;        // ErrorCode
    uint8_t     source;      // ErrorSource
    uint8_t     level;       // ErrorLevel
    uint32_t    timestamp;   // Timestamp en ms (ou compteur simple)
} ErrorEvent;


typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool    blink;           // True = clignotement
    uint16_t blink_period_ms;
} LedConfig;



void ErrorManager_Init(void);

// Gestion des erreurs
void ErrorManager_SetError(ErrorSource src, ErrorCode code, ErrorLevel level);
void ErrorManager_ClearError(ErrorSource src, ErrorCode code);
void ErrorManager_ClearAllErrors(void);

// État système
void ErrorManager_SetState(SystemState state);
SystemState ErrorManager_GetState(void);

ErrorLevel ErrorManager_GetMaxLevel(void);

uint8_t ErrorManager_GetActiveErrorCount(void);

uint8_t ErrorManager_GetHistory(ErrorEvent *buffer, uint8_t max_events);

// Mise à jour de la LED RGB
void ErrorManager_UpdateLed(void);

#endif // ERROR_MANAGER_H