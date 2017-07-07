----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05/22/2017 10:50:19 AM
-- Design Name: 
-- Module Name: concurrent_signal - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
--          MUST Avoid this kind of design when we have a dependence of conccurrent signal
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity concurrent_signal is
    Port ( output    : out integer range 0 to 100000;
           output_1   : out integer range 0 to 100000);
end concurrent_signal;

architecture Behavioral of concurrent_signal is
signal A : integer range 0 to 100000 := 15;
signal B : integer range 0 to 100000 := 23;
begin
    A <= B + 10;
    B <= A + 20;
    output      <= A;
    output_1    <= B;
end Behavioral;
