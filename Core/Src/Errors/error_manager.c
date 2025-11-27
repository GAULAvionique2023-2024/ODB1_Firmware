#include "Errors/error_manager.h"
#include <string.h>


#define ERROR_MANAGER_MAX_ACTIVE    16   // Max erreurs actives simultanées
#define ERROR_MANAGER_HISTORY_SIZE  32   // Historique (8 octets × 32 = 256 octets)
#define LED_BLINK_PERIOD_MS         500  // Période de clignotement par défaut


typedef struct {
    bool        in_use;
    ErrorSource source;
    ErrorCode   code;
    ErrorLevel  level;
} ActiveError;

static ActiveError s_active[ERROR_MANAGER_MAX_ACTIVE];
static ErrorEvent  s_history[ERROR_MANAGER_HISTORY_SIZE];
static uint8_t     s_history_head  = 0;
static uint8_t     s_history_count = 0;

static SystemState s_current_state = STATE_INIT;
static uint32_t    s_last_blink_time = 0;
static bool        s_led_blink_state = false;


static const LedConfig s_led_configs[] = {
    [STATE_INIT]        = {255, 255,   0, true,  1000},  // Jaune clignotant
    [STATE_PREFLIGHT]   = {  0,   0, 255, false,    0},  // Bleu fixe
    [STATE_ARMED]       = {255, 165,   0, true,   500},  // Orange clignotant
    [STATE_FLIGHT]      = {  0, 255,   0, false,    0},  // Vert fixe
    [STATE_RECOVERY]    = {  0, 255,   0, true,  1000},  // Vert clignotant
    [STATE_LANDED]      = {255, 255, 255, false,    0},  // Blanc fixe
    [STATE_ERROR]       = {255, 255,   0, false,    0},  // Jaune fixe
    [STATE_CRITICAL]    = {255,   0,   0, true,   250},  // Rouge clignotant rapide
    [STATE_SAFE_MODE]   = {255,   0, 255, true,   500},  // Magenta clignotant
    [STATE_DEBUG]       = {128,   0, 128, true,  1000},  // Violet clignotant
};


// Logguer un événement dans l'historique
static void log_event(ErrorSource src, ErrorCode code, ErrorLevel level, uint32_t timestamp)
{
    ErrorEvent *e = &s_history[s_history_head];
    e->code      = (uint16_t)code;
    e->source    = (uint8_t)src;
    e->level     = (uint8_t)level;
    e->timestamp = timestamp;
    
    s_history_head = (s_history_head + 1) % ERROR_MANAGER_HISTORY_SIZE;
    
    if (s_history_count < ERROR_MANAGER_HISTORY_SIZE) {
        s_history_count++;
    }
}

// Recalculer l'état système basé sur les erreurs actives
static void recalc_state(void)
{
    ErrorLevel max_level = ERR_LEVEL_INFO;
    
    // Trouver le niveau d'erreur maximum
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (s_active[i].in_use && s_active[i].level > max_level) {
            max_level = s_active[i].level;
        }
    }
    
    // Mettre à jour l'état système selon la sévérité
    // (Ne change pas si déjà en FLIGHT, RECOVERY, LANDED, DEBUG)
    if (s_current_state != STATE_FLIGHT && s_current_state != STATE_RECOVERY && s_current_state != STATE_LANDED && s_current_state != STATE_DEBUG) {
        
        switch (max_level) {
            case ERR_LEVEL_FATAL:
            case ERR_LEVEL_CRITICAL:
                //TODO handle erreur critique
                s_current_state = STATE_SAFE_MODE;
                break;
            case ERR_LEVEL_ERROR:
                s_current_state = STATE_ERROR;
                break;
            case ERR_LEVEL_WARNING:
                break;
            case ERR_LEVEL_INFO:
            default:
                // Si aucune erreur, retourner à PREFLIGHT
                if (s_current_state == STATE_ERROR || s_current_state == STATE_SAFE_MODE) {
                    s_current_state = STATE_PREFLIGHT;
                }
                break;
        }
    }
}


static void update_led_hardware(uint8_t r, uint8_t g, uint8_t b)
{
    // TODO: Implémenter LED
}


void ErrorManager_Init(void)
{
    // Initialiser les erreurs actives
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        s_active[i].in_use = false;
        s_active[i].source = SRC_NONE;
        s_active[i].code   = ERR_OK;
        s_active[i].level  = ERR_LEVEL_INFO;
    }
    
    // Réinitialiser l'historique
    s_history_head  = 0;
    s_history_count = 0;
    
    // État initial
    s_current_state = STATE_INIT;
    s_last_blink_time = 0;
    s_led_blink_state = false;
}

void ErrorManager_SetError(ErrorSource src, ErrorCode code, ErrorLevel level)
{
    // Vérifier si l'erreur existe déjà
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (s_active[i].in_use && s_active[i].source == src && s_active[i].code == code) {
            // Mettre à jour le niveau si plus sévère
            if (level > s_active[i].level) {
                s_active[i].level = level;
            }
            return;
        }
    }
    
    // Trouver un slot libre
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (!s_active[i].in_use) {

            s_active[i].in_use = true;
            s_active[i].source = src;
            s_active[i].code   = code;
            s_active[i].level  = level;
            
            // Logger l'événement
            log_event(src, code, level, 0); // TODO: ajouter timestamp réel
            
            // Recalculer l'état système
            recalc_state();
            return;
        }
    }
    
    // TODO pas de slot libre
}

void ErrorManager_ClearError(ErrorSource src, ErrorCode code)
{
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (s_active[i].in_use && s_active[i].source == src && s_active[i].code == code) {
            
            s_active[i].in_use = false;
            
            // Recalculer l'état
            recalc_state();
            return;
        }
    }
}

void ErrorManager_ClearAllErrors(void)
{
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        s_active[i].in_use = false;
    }
    recalc_state();
}

void ErrorManager_SetState(SystemState state)
{
    s_current_state = state;
}

SystemState ErrorManager_GetState(void)
{
    return s_current_state;
}

ErrorLevel ErrorManager_GetMaxLevel(void)
{
    ErrorLevel max_level = ERR_LEVEL_INFO;
    
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (s_active[i].in_use && s_active[i].level > max_level) {
            max_level = s_active[i].level;
        }
    }
    
    return max_level;
}

uint8_t ErrorManager_GetActiveErrorCount(void)
{
    uint8_t count = 0;
    for (int i = 0; i < ERROR_MANAGER_MAX_ACTIVE; i++) {
        if (s_active[i].in_use) {
            count++;
        }
    }
    return count;
}

uint8_t ErrorManager_GetHistory(ErrorEvent *buffer, uint8_t max_events)
{
    if (buffer == NULL || max_events == 0) {
        return 0;
    }
    
    uint8_t count = (s_history_count < max_events) ? s_history_count : max_events;
    
    // Copier les événements les plus récents
    uint8_t read_idx = (s_history_head + ERROR_MANAGER_HISTORY_SIZE - count) % ERROR_MANAGER_HISTORY_SIZE;
    
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = s_history[read_idx];
        read_idx = (read_idx + 1) % ERROR_MANAGER_HISTORY_SIZE;
    }
    
    return count;
}

void ErrorManager_UpdateLed(void)
{
    const LedConfig *cfg = &s_led_configs[s_current_state];
    
    uint8_t r = cfg->r;
    uint8_t g = cfg->g;
    uint8_t b = cfg->b;
    
    // Gérer le clignotement
    if (cfg->blink) {
        uint32_t current_time = 0; // TODO: utiliser HAL_GetTick()
        
        if (current_time - s_last_blink_time >= cfg->blink_period_ms) {
            s_led_blink_state = !s_led_blink_state;
            s_last_blink_time = current_time;
        }
        
        if (!s_led_blink_state) {
            r = 0;
            g = 0;
            b = 0;
        }
    }
    
    update_led_hardware(r, g, b);
}