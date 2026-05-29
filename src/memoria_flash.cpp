#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h> 
#include <SPI.h> // Biblioteca nativa para controle de hardware SPI

//pinos
const int pinoCS    = 10;
uint32_t end_atual = 0x000000;

// Configuração do SPI (Velocidade, Ordem dos bits e Modo)
SPISettings configuracaoSPI(8000000, MSBFIRST, SPI_MODE0);

// Função para checar o Status Register e garantir que a memória não está ocupada
void aguardar_flash_pronta() {
    digitalWrite(pinoCS, LOW);
    SPI.transfer(0x05); // Instrução Read Status Register-1 (05h)
    while (SPI.transfer(0x00) & 0x01) {
        // Fica aqui enquanto o bit BUSY (bit 0) for 1
    }
    digitalWrite(pinoCS, HIGH);
}

// Ativa o (WEL)
void executar_write_enable() {
    digitalWrite(pinoCS, HIGH);
    delayMicroseconds(1);
    digitalWrite(pinoCS, LOW); 
    delayMicroseconds(1);

    SPI.transfer(0x06); // Instrução Write Enable (06h)

    digitalWrite(pinoCS, HIGH);
    delayMicroseconds(1);
}

//OPERAÇÕES DA MEMÓRIA FLASH

void chip_erase() {
    executar_write_enable(); 
    digitalWrite(pinoCS, LOW); 
    SPI.transfer(0xC7); // Instrução de Chip Erase (C7h)
    digitalWrite(pinoCS, HIGH); 
    
    aguardar_flash_pronta(); 
}

// Retorna true se achar algum byte diferente de 0xFF (já escrito)
bool executar_read(uint32_t endereco) {
    uint8_t A23_A16 = (endereco >> 16) & 0xFF;
    uint8_t A15_A8  = (endereco >> 8) & 0xFF;
    uint8_t A7_A0   = (endereco) & 0xFF;

    digitalWrite(pinoCS, LOW); 
    SPI.transfer(0x03); // Instrução Read (03h)
    
    SPI.transfer(A23_A16);     
    SPI.transfer(A15_A8);      
    SPI.transfer(A7_A0);       
    
    bool possui_dados = false;

    // Varre os 4K bytes
    for(int i = 0; i < 4096; i++) {
        uint8_t dadoRecebido = SPI.transfer(0x00); // Envia um byte vazio para receber o dado da Flash
        if(dadoRecebido != 0xFF) {
            possui_dados = true; 
        }
    }
    
    digitalWrite(pinoCS, HIGH); 
    return possui_dados;
}

void executar_erase_4K_bytes(uint32_t endereco) {
    uint8_t A23_A16 = (endereco >> 16) & 0xFF;
    uint8_t A15_A8  = (endereco >> 8) & 0xFF;
    uint8_t A7_A0   = (endereco) & 0xFF;

    executar_write_enable(); 
    digitalWrite(pinoCS, LOW); 
    SPI.transfer(0x20); // Instrução de Sector Erase 4K (20h)
    
    SPI.transfer(A23_A16); 
    SPI.transfer(A15_A8);  
    SPI.transfer(A7_A0);   
    digitalWrite(pinoCS, HIGH);

    aguardar_flash_pronta(); 
}

// Grava 4K bytes
void executar_page_program_4K(uint32_t endereco_inicial) {
    
    // Divide o setor de 4K em 16 ciclos de escrita de 256 bytes
    for (int page = 0; page < 16; page++) {
        
        // Calcula o endereço inicial da page atual
        uint32_t endereco_page_atual = endereco_inicial + (page * 256);
        
        // Ativa o protocolo de escrita antes de iniciar a page
        executar_write_enable(); 
        
        digitalWrite(pinoCS, LOW); 
        SPI.transfer(0x02); // Instrução Page Program (02h)
        
        // Envia o endereço de 24 bits da page correspondente
        SPI.transfer((endereco_page_atual >> 16) & 0xFF);     
        SPI.transfer((endereco_page_atual >> 8) & 0xFF);      
        SPI.transfer(endereco_page_atual & 0xFF);       
        
        // Envia os 256 bytes sequencialmente um por um
        for (int i = 0; i < 256; i++) {
            uint8_t dado; // Ver melhor como separar esse dado, se vai pegar de .txt sei lá
            SPI.transfer(dado); 
        }
        
        digitalWrite(pinoCS, HIGH); 
        delay(3); // Aguarda a gravação física dos transistores da Flash
    }
}



enum Estados {
    Erase_total,
    Erase_4K_bytes,
    Read,
    Page_Program
};
Estados estado_atual = Erase_total;

void setup() {
    pinMode(pinoCS, OUTPUT);
    digitalWrite(pinoCS, HIGH); 
    
    SPI.begin();
    SPI.beginTransaction(configuracaoSPI);
    
    Serial.begin(9600);
}

void loop() {
    // Cálculo do endereço de verificação (4K à frente)
    uint32_t end_4k_a_frente = end_atual + 4096;

    switch (estado_atual) {
        case Erase_total:
            chip_erase();
            estado_atual = Read; 
            break;
            
        case Read:
            if (executar_read(end_4k_a_frente)) {
                estado_atual = Erase_4K_bytes; 
            } else {
                end_atual += 4096; 
                estado_atual = Page_Program; 
            }
            break;
            
        case Erase_4K_bytes:
            executar_erase_4K_bytes(end_4k_a_frente); 
            estado_atual = Page_Program;
            break;         
            
        case Page_Program:
            executar_page_program_4K(end_atual); 
            
            // Trava o loop para evitar ciclos indesejados de re-escrita
            while(true); 
            break;
    }
}