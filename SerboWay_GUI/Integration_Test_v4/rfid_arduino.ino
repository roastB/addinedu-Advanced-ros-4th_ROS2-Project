#include <SPI.h> 
#include <MFRC522.h> 

const int RST_PIN = 9; 
const int SS_PIN = 10; 

MFRC522 rc522(SS_PIN, RST_PIN); 

const int TOTAL_INDEX =60;
MFRC522::MIFARE_Key key;

void setup() 
{
  Serial.begin(9600); 

  for (int i = 0; i < 6; i++)
  {
    key.keyByte[i] = 0xFF;
  }
} 
MFRC522::StatusCode checkAuth(int index)
{
  MFRC522::StatusCode status = rc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, index, &key, &(rc522.uid));
  return status;
}

MFRC522::StatusCode readData(int index, byte* data)
{
  MFRC522::StatusCode status = checkAuth(index);
  if (status != MFRC522::STATUS_OK)
    {
      return status;
    }

    byte buffer[18]; 
    byte length = 18;
    status = rc522.MIFARE_Read(index, buffer, &length);
    if(status == MFRC522::STATUS_OK)
    {
      memcpy(data, buffer, 4);
    }
    return status;
}

MFRC522::StatusCode writeData(int index, byte* data, int length)
    {
      MFRC522::StatusCode status = checkAuth(index);
      if (status != MFRC522::STATUS_OK)
      {
        return status;
      }
      byte buffer[16]; 
      memset(buffer, 0x00, sizeof(buffer)); 
      memcpy(buffer, data, length); 

      status = rc522.MIFARE_Write(index, buffer, 16); 
      return status;
    }

void loop()
{
  SPI.begin(); 
  rc522.PCD_Init(); 

  int recv_size = 0; 
  char recv_buffer[16]; 

  if (Serial.available() > 0) 
  {
    recv_size = Serial.readBytesUntil('\n', recv_buffer, 16); 
  }

  bool newCard = rc522.PICC_IsNewCardPresent(); 
  bool readCard = rc522.PICC_ReadCardSerial(); 

  if (recv_size > 0)
  {
    //get command 
    char cmd[2]; 
    memset(cmd, 0x00, sizeof(cmd)); 
    memcpy(cmd, recv_buffer, 2); 
    char send_buffer[16]; 
    memset(send_buffer, 0x00, sizeof(send_buffer)); 
    memcpy(send_buffer, cmd, 2); 

    if(strncmp(cmd, "GS", 2)!=0)
    {
      if(memcmp(recv_buffer + 2, rc522.uid.uidByte, 4)!=0)
      {
        memset(send_buffer + 2, 0xFB, 1);
        Serial.write(send_buffer, 3);
        Serial.println();
        return;
      }
    }

    MFRC522::StatusCode status = MFRC522::STATUS_ERROR;
    if (newCard == true && readCard == true)
    {
      if (strncmp(cmd, "GS", 2) == 0)
      {
        memset(send_buffer + 2, MFRC522::STATUS_OK, 1);
        memcpy(send_buffer + 3, rc522.uid.uidByte, 4); 
        Serial.write(send_buffer, 7); 
      }
      else if (strncmp(cmd, "GT", 2) == 0)
      {
        byte total[4];
        memset(total, 0x00, 4); 
        status = readData(TOTAL_INDEX, total);

        memset(send_buffer + 2, status, 1); 
        memcpy(send_buffer + 3, total, 4); 
        Serial.write(send_buffer, 7); 
      }
      else if (strncmp(cmd, "ST", 2) == 0)
      {
        char total[4]; 
        memset(total, 0x00, sizeof(total)); 
        memcpy(total, recv_buffer + 6, 4); 

        status = writeData(TOTAL_INDEX, total, 4); 
        memset (send_buffer + 2, status, 1); 
        Serial.write(send_buffer, 3); 
      }
      else
      {
        memset(send_buffer + 2, 0xFE, 1); 
        Serial.write(send_buffer, 3); 
      }




    }
    else
    {
      memset(send_buffer + 2, 0xFA, 1); 
      Serial.write(send_buffer, 3); 
    }

    Serial.println();

  }

}