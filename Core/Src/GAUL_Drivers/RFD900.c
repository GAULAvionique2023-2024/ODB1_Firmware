/*
 * RFD900.c
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

uint8_t ICM20602_Init(RFD900 *dev, UART_HandleTypeDef *UART_Handle);

// Low level fonctions
HAL_StatusTypeDef RFD900_ReadRegister(RFD900 *dev, uint8_t *data, uint8_t *reg, uint8_t length) {


}

// High Level functions
HAL_StatusTypeDef RFD900_TransmitPacket(RFD900 *dev, uint8_t *data, uint8_t *reg, uint8_t length) {


}

HAL_StatusTypeDef RFD900_ReceivePacket(RFD900 *dev, uint8_t *data, uint8_t *reg) {


}
