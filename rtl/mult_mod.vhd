---------------------------------------------------------------------
-- TITLE: Multiplication and Division Unit
-- AUTHORS: Steve Rhoads (rhoadss@yahoo.com)
-- DATE CREATED: 1/31/01
-- FILENAME: mult.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    Implements the multiplication and division unit in 3 clocks.
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use IEEE.std_logic_arith.all;
use work.mlite_pack.all;

entity mult_mod is
   generic(mult_type  : string := "DEFAULT");
   port(clk       : in std_logic;
        reset_in  : in std_logic;
        a, b      : in std_logic_vector(31 downto 0);
        mult_func : in mult_function_type;
        c_mult    : out std_logic_vector(31 downto 0);
        pause_out : out std_logic);
end; --entity mult

architecture logic of mult_mod is

	component CSA is
	port(
		in1 : in std_logic_vector(31 downto 0);
		in2 : in std_logic_vector(31 downto 0);
		out1 : out std_logic_vector(62 downto 0);
		out2 : out std_logic_vector(62 downto 0));
	end component;
	
   constant MODE_MULT : std_logic := '1';
   constant MODE_DIV  : std_logic := '0';

   signal mode_reg    : std_logic;
   signal CSA_out1, CSA_out2 : std_logic_vector(62 downto 0);
   signal adder_in1, adder_in2 : std_logic_vector(63 downto 0);
   signal negate_reg  : std_logic;
   signal count_reg   : std_logic_vector(5 downto 0);
   signal aa_reg      : std_logic_vector(31 downto 0);
   signal bb_reg      : std_logic_vector(31 downto 0);
   signal upper_reg   : std_logic_vector(31 downto 0);
   signal lower_reg   : std_logic_vector(31 downto 0);

   signal a_neg       : std_logic_vector(31 downto 0);
   signal b_neg       : std_logic_vector(31 downto 0);
   signal sum         : std_logic_vector(32 downto 0);
   signal result	  : std_logic_vector(63 downto 0);
   signal product	  : std_logic_vector(63 downto 0);
   
   
begin
 
   -- Result
   c_mult <= lower_reg when mult_func = MULT_READ_LO and (negate_reg = '0' or mode_reg = MODE_MULT) else 
             bv_negate(lower_reg) when mult_func = MULT_READ_LO 
                and negate_reg = '1' else
             upper_reg when mult_func = MULT_READ_HI and (negate_reg = '0' or mode_reg = MODE_MULT)  else 
             bv_negate(upper_reg) when mult_func = MULT_READ_HI 
                and negate_reg = '1' else
             ZERO;
   pause_out <= '1' when (count_reg /= "000000") and 
             (mult_func = MULT_READ_LO or mult_func = MULT_READ_HI) else '0';
   
  

   product <= bv_negate(result) when negate_reg = '1' else result;
   
   -- ABS and remainder signals
   a_neg <= bv_negate(a);
   b_neg <= bv_negate(b);
   sum <= bv_adder(upper_reg, aa_reg, mode_reg);
   adder_in1(63) <= '0';
   adder_in2(63) <= '0';
   lbl_CSA: CSA port map(aa_reg, bb_reg, CSA_out1, CSA_out2);
	result <= bv_adder(adder_in1, adder_in2, '1')(63 downto 0); 
	process(clk, reset_in, a, b, a_neg, b_neg, CSA_out1, CSA_out2, product, mode_reg, upper_reg, lower_reg, count_reg)
	begin
	if reset_in = '1' then
		adder_in1(62 downto 0) <= (others => '0');
		adder_in2(62 downto 0) <=(others => '0');
		aa_reg <= (others => '0');
		bb_reg <= (others => '0');
		count_reg <= "000000";
		negate_reg <= '0';
		upper_reg <= (others => '0');
        lower_reg <= (others => '0');
	else
		if rising_edge(clk) then
			adder_in1(62 downto 0) <= CSA_out1;
			adder_in2(62 downto 0) <= CSA_out2;
			
			case mult_func is
			when MULT_WRITE_LO =>
               lower_reg <= a;
               negate_reg <= '0';
            when MULT_WRITE_HI =>
               upper_reg <= a;
               negate_reg <= '0';
            when MULT_MULT =>
			   mode_reg <= MODE_MULT;
               aa_reg <= a;
               bb_reg <= b;
               count_reg <= "000010"; --Amount of clock cycles a multiplication takes
			   negate_reg <= '0';
            when MULT_SIGNED_MULT =>
			   mode_reg <= MODE_MULT;
               if b(31) = '0' then
                  bb_reg <= b;
               else
                  bb_reg <= b_neg;
               end if;
               if a(31) = '0' then
                  aa_reg <= a;
               else
                  aa_reg <= a_neg; 
               end if;
			   negate_reg <= a(31) xor b(31);
               count_reg <= "000010"; --Amount of clock cycles a multiplication takes
			when MULT_DIVIDE =>
               mode_reg <= MODE_DIV;
               aa_reg <= b(0) & ZERO(30 downto 0);
               bb_reg <= b;
               upper_reg <= a;
               count_reg <= "100000"; --Amount of clock cycles a division takes
               negate_reg <= '0';
            when MULT_SIGNED_DIVIDE =>
               mode_reg <= MODE_DIV;
               if b(31) = '0' then
                  aa_reg(31) <= b(0);
                  bb_reg <= b;
               else
                  aa_reg(31) <= b_neg(0);
                  bb_reg <= b_neg;
               end if;
               if a(31) = '0' then
                  upper_reg <= a;
               else
                  upper_reg <= a_neg;
               end if;
               aa_reg(30 downto 0) <= ZERO(30 downto 0);
               count_reg <= "100000"; --Amount of clock cycles a division takes
               negate_reg <= a(31) xor b(31);
            when others =>
               if count_reg /= "000000" then
                  if mode_reg = MODE_MULT then
                     -- Multiplication
                     upper_reg <= product(63 downto 32);
					 lower_reg <= product(31 downto 0);
                  else   
                     -- Division
                     if sum(32) = '0' and aa_reg /= ZERO and 
                           bb_reg(31 downto 1) = ZERO(31 downto 1) then
                        upper_reg <= sum(31 downto 0);
                        lower_reg(0) <= '1';
                     else
                        lower_reg(0) <= '0';
                     end if;
                     aa_reg <= bb_reg(1) & aa_reg(31 downto 1);
                     lower_reg(31 downto 1) <= lower_reg(30 downto 0);
                     bb_reg <= '0' & bb_reg(31 downto 1);
                  end if;
                  count_reg <= count_reg - 1;
               end if; --count
         end case;
	end if;
	end if;
	end process;
end; --architecture logic
