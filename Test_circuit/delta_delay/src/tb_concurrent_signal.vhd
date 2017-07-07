----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05/22/2017 10:52:33 AM
-- Design Name: 
-- Module Name: tb_concurrent_signal - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use work.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity tb_concurrent_signal is
--  Port ( );
end tb_concurrent_signal;

architecture Behavioral of tb_concurrent_signal is
signal output, output_1   : integer range 0 to 100000;
begin
 UUT : entity work.concurrent_signal port map (output, output_1);
end Behavioral;
