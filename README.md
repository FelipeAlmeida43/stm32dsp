
Projeto STM32H743 - Leitura de Sinal Senoidal com Filtro IIR
Este repositório contém um projeto para a plataforma STM32H743, que visa a leitura de um sinal senoidal no pino PB1, aplicando um filtro IIR (Infinite Impulse Response).

Descrição
O objetivo deste projeto é demonstrar a capacidade da STM32H743 em ler um sinal analógico senoidal e aplicar um filtro IIR para processamento do sinal. O pino PB1 é utilizado para a entrada do sinal analógico.

Requisitos
STM32CubeIDE
STM32H743ZI-Nucleo Board (ou similar)
Conhecimento básico em programação embarcada e STM32 HAL
Configuração do Projeto
Clone este repositório:

bash
Copy code
git clone https://github.com/seu-username/stm32h743-senoidal-iir.git
Abra o projeto no STM32CubeIDE.

Configure as opções do projeto conforme necessário.

Compile o projeto e faça o upload para a placa.

Pinagem
PB1: Entrada do sinal senoidal
Filtro IIR
O filtro IIR implementado no projeto é configurado para atender aos requisitos específicos do sinal senoidal utilizado. As configurações podem ser ajustadas no arquivo filtro_iir.c.

c
Copy code
// Exemplo de configuração do filtro IIR
#define IIR_NUM_STAGES 2
#define IIR_COEFFICIENTS {0.1, 0.9}
Contribuições
Contribuições são bem-vindas! Sinta-se à vontade para abrir problemas (issues) ou enviar pull requests com melhorias.

Licença
Este projeto é distribuído sob a Licença MIT.
