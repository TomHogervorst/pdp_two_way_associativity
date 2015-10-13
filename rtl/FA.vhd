library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;

entity FA is
	port(in1 : in std_logic;
		in2 : in std_logic;
		c_in : in std_logic;
		sum : out std_logic;
		c_out : out std_logic);
	end;
	
architecture behavioural of FA is
	
	signal xor1, and1, and2 : std_logic;

begin

	xor1 <= in1 xor in2;
	and1 <= in1 and in2;
	and2 <= c_in and xor1;
	sum <= xor1 xor c_in;
	c_out <= and1 or and2;

end;