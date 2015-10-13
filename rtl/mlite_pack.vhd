---------------------------------------------------------------------
-- TITLE: Plasma Misc. Package
-- AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
-- DATE CREATED: 2/15/01
-- FILENAME: mlite_pack.vhd
-- PROJECT: Plasma CPU core
-- COPYRIGHT: Software placed into the public domain by the author.
--    Software 'as is' without warranty.  Author liable for nothing.
-- DESCRIPTION:
--    Data types, constants, and add functions needed for the Plasma CPU.
---------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

package mlite_pack is
   constant ZERO          : std_logic_vector(31 downto 0) :=
      "00000000000000000000000000000000";
   constant ONES          : std_logic_vector(31 downto 0) :=
      "11111111111111111111111111111111";
   --make HIGH_Z equal to ZERO if compiler complains
   constant HIGH_Z        : std_logic_vector(31 downto 0) :=
      "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
  
   subtype alu_function_type is std_logic_vector(3 downto 0);
   constant ALU_NOTHING   : alu_function_type := "0000";
   constant ALU_ADD       : alu_function_type := "0001";
   constant ALU_SUBTRACT  : alu_function_type := "0010";
   constant ALU_LESS_THAN : alu_function_type := "0011";
   constant ALU_LESS_THAN_SIGNED : alu_function_type := "0100";
   constant ALU_OR        : alu_function_type := "0101";
   constant ALU_AND       : alu_function_type := "0110";
   constant ALU_XOR       : alu_function_type := "0111";
   constant ALU_NOR       : alu_function_type := "1000";

   subtype shift_function_type is std_logic_vector(1 downto 0);
   constant SHIFT_NOTHING        : shift_function_type := "00";
   constant SHIFT_LEFT_UNSIGNED  : shift_function_type := "01";
   constant SHIFT_RIGHT_SIGNED   : shift_function_type := "11";
   constant SHIFT_RIGHT_UNSIGNED : shift_function_type := "10";

   subtype mult_function_type is std_logic_vector(3 downto 0);
   constant MULT_NOTHING       : mult_function_type := "0000";
   constant MULT_READ_LO       : mult_function_type := "0001";
   constant MULT_READ_HI       : mult_function_type := "0010";
   constant MULT_WRITE_LO      : mult_function_type := "0011";
   constant MULT_WRITE_HI      : mult_function_type := "0100";
   constant MULT_MULT          : mult_function_type := "0101";
   constant MULT_SIGNED_MULT   : mult_function_type := "0110";
   constant MULT_DIVIDE        : mult_function_type := "0111";
   constant MULT_SIGNED_DIVIDE : mult_function_type := "1000";

   subtype a_source_type is std_logic_vector(1 downto 0);
   constant A_FROM_REG_SOURCE : a_source_type := "00";
   constant A_FROM_IMM10_6    : a_source_type := "01";
   constant A_FROM_PC         : a_source_type := "10";

   subtype b_source_type is std_logic_vector(1 downto 0);
   constant B_FROM_REG_TARGET : b_source_type := "00";
   constant B_FROM_IMM        : b_source_type := "01";
   constant B_FROM_SIGNED_IMM : b_source_type := "10";
   constant B_FROM_IMMX4      : b_source_type := "11";

   subtype c_source_type is std_logic_vector(2 downto 0);
   constant C_FROM_NULL       : c_source_type := "000";
   constant C_FROM_ALU        : c_source_type := "001";
   constant C_FROM_SHIFT      : c_source_type := "001"; --same as alu
   constant C_FROM_MULT       : c_source_type := "001"; --same as alu
   constant C_FROM_MEMORY     : c_source_type := "010";
   constant C_FROM_PC         : c_source_type := "011";
   constant C_FROM_PC_PLUS4   : c_source_type := "100";
   constant C_FROM_IMM_SHIFT16: c_source_type := "101";
   constant C_FROM_REG_SOURCEN: c_source_type := "110";

   subtype pc_source_type is std_logic_vector(1 downto 0);
   constant FROM_INC4       : pc_source_type := "00";
   constant FROM_OPCODE25_0 : pc_source_type := "01";
   constant FROM_BRANCH     : pc_source_type := "10";
   constant FROM_LBRANCH    : pc_source_type := "11";

   subtype branch_function_type is std_logic_vector(2 downto 0);
   constant BRANCH_LTZ : branch_function_type := "000";
   constant BRANCH_LEZ : branch_function_type := "001";
   constant BRANCH_EQ  : branch_function_type := "010";
   constant BRANCH_NE  : branch_function_type := "011";
   constant BRANCH_GEZ : branch_function_type := "100";
   constant BRANCH_GTZ : branch_function_type := "101";
   constant BRANCH_YES : branch_function_type := "110";
   constant BRANCH_NO  : branch_function_type := "111";

   -- mode(32=1,16=2,8=3), signed, write
   subtype mem_source_type is std_logic_vector(3 downto 0);
   constant MEM_FETCH   : mem_source_type := "0000";
   constant MEM_READ32  : mem_source_type := "0100";
   constant MEM_WRITE32 : mem_source_type := "0101";
   constant MEM_READ16  : mem_source_type := "1000";
   constant MEM_READ16S : mem_source_type := "1010";
   constant MEM_WRITE16 : mem_source_type := "1001";
   constant MEM_READ8   : mem_source_type := "1100";
   constant MEM_READ8S  : mem_source_type := "1110";
   constant MEM_WRITE8  : mem_source_type := "1101";

   function bv_adder(a     : in std_logic_vector;
                     b     : in std_logic_vector;
                     do_add: in std_logic) return std_logic_vector;
  -- function custom_64bit_carrylookahead(a      : in std_logic_vector;
                                       -- b      : in std_logic_vector;
                                       -- do_add : in std_logic) return std_logic_vector;
    -- function custom_32bit_carrylookahead(a      : in std_logic_vector;
                                         -- b      : in std_logic_vector;
                                         -- do_add : in std_logic) return std_logic_vector;
   function bv_negate(a : in std_logic_vector) return std_logic_vector;
   function bv_increment(a : in std_logic_vector(31 downto 2)
                         ) return std_logic_vector;
   function bv_inc(a : in std_logic_vector
                  ) return std_logic_vector;

   -- For Altera
   COMPONENT lpm_ram_dp
      generic (
         LPM_WIDTH : natural;    -- MUST be greater than 0
         LPM_WIDTHAD : natural;    -- MUST be greater than 0
         LPM_NUMWORDS : natural := 0;
         LPM_INDATA : string := "REGISTERED";
         LPM_OUTDATA : string := "REGISTERED";
         LPM_RDADDRESS_CONTROL : string := "REGISTERED";
         LPM_WRADDRESS_CONTROL : string := "REGISTERED";
         LPM_FILE : string := "UNUSED";
         LPM_TYPE : string := "LPM_RAM_DP";
         USE_EAB  : string := "OFF";
         INTENDED_DEVICE_FAMILY  : string := "UNUSED";
         RDEN_USED  : string := "TRUE";
         LPM_HINT : string := "UNUSED");
      port (
         RDCLOCK   : in std_logic := '0';
         RDCLKEN   : in std_logic := '1';
         RDADDRESS : in std_logic_vector(LPM_WIDTHAD-1 downto 0);
         RDEN      : in std_logic := '1';
         DATA      : in std_logic_vector(LPM_WIDTH-1 downto 0);
         WRADDRESS : in std_logic_vector(LPM_WIDTHAD-1 downto 0);
         WREN      : in std_logic;
         WRCLOCK   : in std_logic := '0';
         WRCLKEN   : in std_logic := '1';
         Q         : out std_logic_vector(LPM_WIDTH-1 downto 0));
   END COMPONENT;

   -- For Altera
   component LPM_RAM_DQ
      generic (
         LPM_WIDTH    : natural;    -- MUST be greater than 0
         LPM_WIDTHAD  : natural;    -- MUST be greater than 0
         LPM_NUMWORDS : natural := 0;
         LPM_INDATA   : string := "REGISTERED";
         LPM_ADDRESS_CONTROL: string := "REGISTERED";
         LPM_OUTDATA  : string := "REGISTERED";
         LPM_FILE     : string := "UNUSED";
         LPM_TYPE     : string := "LPM_RAM_DQ";
         USE_EAB      : string := "OFF";
         INTENDED_DEVICE_FAMILY  : string := "UNUSED";
         LPM_HINT     : string := "UNUSED");
		port (
         DATA     : in std_logic_vector(LPM_WIDTH-1 downto 0);
         ADDRESS  : in std_logic_vector(LPM_WIDTHAD-1 downto 0);
         INCLOCK  : in std_logic := '0';
         OUTCLOCK : in std_logic := '0';
         WE       : in std_logic;
         Q        : out std_logic_vector(LPM_WIDTH-1 downto 0));
   end component;
	
   -- For Xilinx Virtex-5
   component RAM32X1D 
      -- synthesis translate_off 
      generic (INIT : bit_vector := X"00000000"); 
      -- synthesis translate_on 
      port (DPO   : out STD_ULOGIC; 
            SPO   : out STD_ULOGIC; 
            A0    : in STD_ULOGIC; 
            A1    : in STD_ULOGIC; 
            A2    : in STD_ULOGIC; 
            A3    : in STD_ULOGIC; 
            A4    : in STD_ULOGIC; 
            D     : in STD_ULOGIC; 
            DPRA0 : in STD_ULOGIC; 
            DPRA1 : in STD_ULOGIC; 
            DPRA2 : in STD_ULOGIC; 
            DPRA3 : in STD_ULOGIC; 
            DPRA4 : in STD_ULOGIC; 
            WCLK  : in STD_ULOGIC; 
            WE    : in STD_ULOGIC); 
   end component; 
	
   component pc_next
      port(clk         : in std_logic;
           reset_in    : in std_logic;
           pc_new      : in std_logic_vector(31 downto 2);
           take_branch : in std_logic;
           pause_in    : in std_logic;
           opcode25_0  : in std_logic_vector(25 downto 0);
           pc_source   : in pc_source_type;
           pc_future   : out std_logic_vector(31 downto 2);
           pc_current  : out std_logic_vector(31 downto 2);
           pc_plus4    : out std_logic_vector(31 downto 2));
   end component;

   component mem_ctrl
      port(clk          : in std_logic;
           reset_in     : in std_logic;
           pause_in     : in std_logic;
           nullify_op   : in std_logic;
           address_pc   : in std_logic_vector(31 downto 2);
           opcode_out   : out std_logic_vector(31 downto 0);

           address_in   : in std_logic_vector(31 downto 0);
           mem_source   : in mem_source_type;
           data_write   : in std_logic_vector(31 downto 0);
           data_read    : out std_logic_vector(31 downto 0);
           pause_out    : out std_logic;

           address_next : out std_logic_vector(31 downto 2);
           byte_we_next : out std_logic_vector(3 downto 0);

           address      : out std_logic_vector(31 downto 2);
           byte_we      : out std_logic_vector(3 downto 0);
           data_w       : out std_logic_vector(31 downto 0);
           data_r       : in std_logic_vector(31 downto 0));
   end component;

   component control 
      port(opcode       : in  std_logic_vector(31 downto 0);
           intr_signal  : in  std_logic;
           rs_index     : out std_logic_vector(5 downto 0);
           rt_index     : out std_logic_vector(5 downto 0);
           rd_index     : out std_logic_vector(5 downto 0);
           imm_out      : out std_logic_vector(15 downto 0);
           alu_func     : out alu_function_type;
           shift_func   : out shift_function_type;
           mult_func    : out mult_function_type;
           branch_func  : out branch_function_type;
           a_source_out : out a_source_type;
           b_source_out : out b_source_type;
           c_source_out : out c_source_type;
           pc_source_out: out pc_source_type;
           mem_source_out:out mem_source_type;
           exception_out: out std_logic);
   end component;

   component reg_bank
      port(clk            : in  std_logic;
           reset_in       : in  std_logic;
           pause          : in  std_logic;
           rs_index       : in  std_logic_vector(5 downto 0);
           rt_index       : in  std_logic_vector(5 downto 0);
           rd_index       : in  std_logic_vector(5 downto 0);
           reg_source_out : out std_logic_vector(31 downto 0);
           reg_target_out : out std_logic_vector(31 downto 0);
           reg_dest_new   : in  std_logic_vector(31 downto 0);
           intr_enable    : out std_logic);
   end component;

   component bus_mux 
      port(imm_in       : in  std_logic_vector(15 downto 0);
           reg_source   : in  std_logic_vector(31 downto 0);
           a_mux        : in  a_source_type;
           a_out        : out std_logic_vector(31 downto 0);

           reg_target   : in  std_logic_vector(31 downto 0);
           b_mux        : in  b_source_type;
           b_out        : out std_logic_vector(31 downto 0);

           c_bus        : in  std_logic_vector(31 downto 0);
           c_memory     : in  std_logic_vector(31 downto 0);
           c_pc         : in  std_logic_vector(31 downto 2);
           c_pc_plus4   : in  std_logic_vector(31 downto 2);
           c_mux        : in  c_source_type;
           reg_dest_out : out std_logic_vector(31 downto 0);

           branch_func  : in  branch_function_type;
           take_branch  : out std_logic);
   end component;

   component alu
      generic(alu_type  : string := "DEFAULT");
      port(a_in         : in  std_logic_vector(31 downto 0);
           b_in         : in  std_logic_vector(31 downto 0);
           alu_function : in  alu_function_type;
           c_alu        : out std_logic_vector(31 downto 0));
   end component;

   component shifter
      generic(shifter_type : string := "DEFAULT" );
      port(value        : in  std_logic_vector(31 downto 0);
           shift_amount : in  std_logic_vector(4 downto 0);
           shift_func   : in  shift_function_type;
           c_shift      : out std_logic_vector(31 downto 0));
   end component;

   component mult_mod
      generic(mult_type  : string := "DEFAULT"); 
      port(clk       : in  std_logic;
           reset_in  : in  std_logic;
           a, b      : in  std_logic_vector(31 downto 0);
           mult_func : in  mult_function_type;
           c_mult    : out std_logic_vector(31 downto 0);
           pause_out : out std_logic); 
   end component;

   component pipeline
      port(clk            : in  std_logic;
           reset          : in  std_logic;
           a_bus          : in  std_logic_vector(31 downto 0);
           a_busD         : out std_logic_vector(31 downto 0);
           b_bus          : in  std_logic_vector(31 downto 0);
           b_busD         : out std_logic_vector(31 downto 0);
           alu_func       : in  alu_function_type;
           alu_funcD      : out alu_function_type;
           shift_func     : in  shift_function_type;
           shift_funcD    : out shift_function_type;
           mult_func      : in  mult_function_type;
           mult_funcD     : out mult_function_type;
           reg_dest       : in  std_logic_vector(31 downto 0);
           reg_destD      : out std_logic_vector(31 downto 0);
           rd_index       : in  std_logic_vector(5 downto 0);
           rd_indexD      : out std_logic_vector(5 downto 0);

           rs_index       : in  std_logic_vector(5 downto 0);
           rt_index       : in  std_logic_vector(5 downto 0);
           pc_source      : in  pc_source_type;
           mem_source     : in  mem_source_type;
           a_source       : in  a_source_type;
           b_source       : in  b_source_type;
           c_source       : in  c_source_type;
           c_bus          : in  std_logic_vector(31 downto 0);
           pause_any      : in  std_logic;
           pause_pipeline : out std_logic);
   end component;

   component mlite_cpu
      generic(
              mult_type       : string := "DEFAULT";
              shifter_type    : string := "DEFAULT";
              alu_type        : string := "DEFAULT"
             );
      port(clk         : in std_logic;
           reset_in    : in std_logic;
           intr_in     : in std_logic;

           address_next : out std_logic_vector(31 downto 2); --for synch ram
           byte_we_next : out std_logic_vector(3 downto 0); 

           address      : out std_logic_vector(31 downto 2);
           byte_we      : out std_logic_vector(3 downto 0);
           data_w       : out std_logic_vector(31 downto 0);
           data_r       : in std_logic_vector(31 downto 0);
           mem_pause    : in std_logic);
   end component;

   component cache
      port(clk            		: in  std_logic;
           reset          		: in  std_logic;
           address_next   		: in  std_logic_vector(31 downto 2);
           byte_we_next   		: in  std_logic_vector(3 downto 0);
           cpu_address    		: in  std_logic_vector(31 downto 2);
           mem_busy       		: in  std_logic;
		   
		   cache_ram_enable  	: in  std_logic;
		   cache_ram_byte_we 	: in  std_logic_vector(3 downto 0);
		   cache_ram_address 	: in  std_logic_vector(31 downto 2);
		   cache_ram_data_w  	: in  std_logic_vector(31 downto 0);
		   cache_ram_data_r  	: out std_logic_vector(31 downto 0);

           cache_access   		: out std_logic;   --access 4KB cache
           cache_checking 		: out std_logic;   --checking if cache hit
           cache_miss     		: out std_logic);  --cache miss
   end component; --cache

   component cache_ram
      generic(block_count : integer := 1);
      port(clk               : in std_logic;
           enable            : in std_logic;
           write_byte_enable : in std_logic_vector(3 downto 0);
           address           : in std_logic_vector(31 downto 2);
           data_write        : in std_logic_vector(31 downto 0);
           data_read         : out std_logic_vector(31 downto 0));
   end component; --ram
   
   component boot_ram
      generic(block_count : integer := 1);
      port(clk               : in std_logic;
           enable            : in std_logic;
           write_byte_enable : in std_logic_vector(3 downto 0);
           address           : in std_logic_vector(31 downto 2);
           data_write        : in std_logic_vector(31 downto 0);
           data_read         : out std_logic_vector(31 downto 0));
   end component; --boot_ram
   
   component uart
      generic(log_file : string := "UNUSED");
      port(clk          : in std_logic;
           reset        : in std_logic;
           enable_read  : in std_logic;
           enable_write : in std_logic;
           data_in      : in std_logic_vector(7 downto 0);
           data_out     : out std_logic_vector(7 downto 0);
           uart_read    : in std_logic;
           uart_write   : out std_logic;
           busy_write   : out std_logic;
           data_avail   : out std_logic);
   end component; --uart

   component plasma
      generic(
              log_file    : string := "UNUSED";
              use_cache   : std_logic := '0');
      port(clk          : in std_logic;
           reset        : in std_logic;
           uart_write   : out std_logic;
           uart_read    : in std_logic;
   
           address      : out std_logic_vector(31 downto 2);
           byte_we      : out std_logic_vector(3 downto 0); 
           data_write   : out std_logic_vector(31 downto 0);
           data_read    : in std_logic_vector(31 downto 0);
           mem_pause_in : in std_logic;
           no_ddr_start : out std_logic;
           no_ddr_stop  : out std_logic;
        
           gpio0_out    : out std_logic_vector(31 downto 0);
           gpioA_in     : in std_logic_vector(31 downto 0));
   end component; --plasma

   component ddr_ctrl
      port(clk      : in std_logic;
           clk_2x   : in std_logic;
           reset_in : in std_logic;

           address  : in std_logic_vector(25 downto 2);
           byte_we  : in std_logic_vector(3 downto 0);
           data_w   : in std_logic_vector(31 downto 0);
           data_r   : out std_logic_vector(31 downto 0);
           active   : in std_logic;
           no_start : in std_logic;
           no_stop  : in std_logic;
           pause    : out std_logic;

           SD_CK_P  : out std_logic;     --clock_positive
           SD_CK_N  : out std_logic;     --clock_negative
           SD_CKE   : out std_logic;     --clock_enable

           SD_BA    : out std_logic_vector(1 downto 0);  --bank_address
           SD_A     : out std_logic_vector(12 downto 0); --address(row or col)
           SD_CS    : out std_logic;     --chip_select
           SD_RAS   : out std_logic;     --row_address_strobe
           SD_CAS   : out std_logic;     --column_address_strobe
           SD_WE    : out std_logic;     --write_enable

           SD_DQ    : inout std_logic_vector(15 downto 0); --data
           SD_UDM   : out std_logic;     --upper_byte_enable
           SD_UDQS  : inout std_logic;   --upper_data_strobe
           SD_LDM   : out std_logic;     --low_byte_enable
           SD_LDQS  : inout std_logic);  --low_data_strobe
   end component; --ddr
   
end; --package mlite_pack


package body mlite_pack is


function calculateCarries(propogates : in std_logic_vector; --length 4
                          generates  : in std_logic_vector; --length 4 
                          carryIn    : in std_logic;
                          offset     : integer) return std_logic_vector is
    variable result : std_logic_vector(3 downto 0);
begin
    result(0) := carryIn;
    result(1) := generates(offset) 
                  or (propogates(offset) and carryIn);
    result(2) := generates(offset+1) 
                  or (propogates(offset+1) and generates(offset)) 
                  or (propogates(offset+1) and propogates(offset) and carryIn);
    result(3) := generates(offset+2) 
                  or (propogates(offset+2) and generates(offset+1))
                  or (propogates(offset+2) and propogates(offset+1) and generates(offset)) 
                  or (propogates(offset+2) and propogates(offset+1) and propogates(offset+0) and carryIn);
    return result;
end; --function

function combineToGenerate(p      : in std_logic_vector; -- the propagate-bits
                           g      : in std_logic_vector; -- the generate-bits
                           offset : integer) return std_logic is

begin
  return g(offset + 3)
                  or (g(offset + 2) and p(offset + 3))
                  or (g(offset + 1) and p(offset + 2) and p(offset + 3))
                  or (g(offset) and p(offset + 1) and p(offset + 2) and p(offset + 3));
end; --function


function custom_64bit_carrylookahead(a     : in std_logic_vector;
                                     b     : in std_logic_vector;
                                     do_add: in std_logic) return std_logic_vector is
   variable carry_in   : std_logic;
   variable carry_out  : std_logic;
   variable bb         : std_logic_vector(63 downto 0);
   variable result     : std_logic_vector(64 downto 0);
   --different levels of propegating and generating bits
   variable p1 : std_logic_vector(63 downto 0);
   variable g1  : std_logic_vector(63 downto 0);
   variable p2 : std_logic_vector(15 downto 0);
   variable g2 : std_logic_vector(15 downto 0);
   variable p3 : std_logic_vector(3 downto 0);
   variable g3 : std_logic_vector(3 downto 0);
   --calculate the carries for blocks of 16, 4 and 1 bit(s) respectively
   variable calculatedCarriesL3 : std_logic_vector(3 downto 0);
   variable calculatedCarriesL2 : std_logic_vector(15 downto 0);
   variable calculatedCarriesL1 : std_logic_vector(63 downto 0);
begin
   if do_add = '1' then
      bb := b;
      carry_in := '0';
   else
      bb := not b;
      carry_in := '1';
   end if;
   
   --Calculate the generate and propagate valuesof each bit-position
   for index in 0 to 63 loop
     p1(index) := a(index) xor bb(index);
     g1(index) := a(index) and bb(index);
   end loop;
   
   --Calculate the generate and propagate values of groups of 4 bits
   for index in 0 to 15 loop
     p2(index) := p1(index * 4) and p1(index * 4 + 1) and p1(index * 4 + 2) and p1(index * 4 + 3);
     g2(index) := combineToGenerate(p1, g1, index * 4);
   end loop;
   
   --Calculate the generate and propagate values of groups of 16 bits from the groups of 4
   for index in 0 to 3 loop
     p3(index) := p2(index * 4) and p2(index * 4 + 1) and p2(index * 4 + 2) and p2(index * 4 + 3);
     g3(index) := combineToGenerate(p2, g2, index * 4); 
   end loop;
   
   --Calculate the carries in 3 steps. First calculate the carries that come into each group of 16 bits:
   calculatedCarriesL3(3 downto 0) := calculateCarries(p3, g3, carry_in, 0);
   
   --Then calculate the carries for each group of 4 bits.
   for index in 0 to 3 loop
    calculatedCarriesL2( (4*index)+3 downto (4*index)) := calculateCarries(p2, g2, calculatedCarriesL3(index), 4*index);
   end loop;
   
   --Finally calculate the carries for each individual bit.
   for index in 0 to 15 loop
    calculatedCarriesL1( (4*index)+3 downto (4*index)) := calculateCarries(p1, g1, calculatedCarriesL2(index), 4*index);
   end loop;
   
   --Use the carries here to do the local addition.
   for index in 0 to 63 loop
    result(index) := calculatedCarriesL1(index) xor a(index) xor bb(index);
   end loop;
   
   carry_out := combineToGenerate(p3, g3, 0)
                or (p3(0) and p3(1) and p3(2) and p3(3) and carry_in); 
   
   result(a'length) := carry_out xnor do_add;
   return result;
end; --function

function custom_32bit_carrylookahead(a     : in std_logic_vector;
                                     b     : in std_logic_vector;
                                     do_add: in std_logic) return std_logic_vector is
   variable carry_in   : std_logic;
   variable carry_out  : std_logic;
   variable bb         : std_logic_vector(31 downto 0);
   variable result     : std_logic_vector(32 downto 0);
   --different levels of propegating and generating
   variable p1 : std_logic_vector(31 downto 0);
   variable g1  : std_logic_vector(31 downto 0);
   variable p2 : std_logic_vector(7 downto 0);
   variable g2 : std_logic_vector(7 downto 0);
   variable p3 : std_logic_vector(1 downto 0);
   variable g3 : std_logic_vector(1 downto 0);
   variable calculatedCarriesTemp : std_logic_vector(7 downto 0);
   variable calculatedCarries : std_logic_vector(31 downto 0);
begin
   if do_add = '1' then
      bb := b;
      carry_in := '0';
   else
      bb := not b;
      carry_in := '1';
   end if;
   
   --Calculate the generate and propagate valuesof each bit-position
   for index in 0 to 31 loop
     p1(index) := a(index) xor bb(index);
     g1(index) := a(index) and bb(index);
   end loop;
   
   --Calculate the generate and propagate values of groups of 4 bits
   for index in 0 to 7 loop
     p2(index) := p1(index * 4) and p1(index * 4 + 1) and p1(index * 4 + 2) and p1(index * 4 + 3);
     g2(index) := combineToGenerate(p1, g1, index * 4);
   end loop;
   
   --Calculate the generate and propagate values of groups of 16 bits from the groups of 4
   for index in 0 to 1 loop
     p3(index) := p2(index * 4) and p2(index * 4 + 1) and p2(index * 4 + 2) and p2(index * 4 + 3);
     g3(index) := combineToGenerate(p2, g2, index * 4);
   end loop;
   
   --Calculate the carries in 2 steps. First calculate the carries that come into each group of 4 bits:
   calculatedCarriesTemp(3 downto 0) := calculateCarries(p2, g2, carry_in, 0);
   calculatedCarriesTemp(7 downto 4) := calculateCarries(p2, g2, g3(0) or (p3(0) and carry_in), 4);
   --Now calculate the carries for each individual bit.
   for index in 0 to 7 loop
    calculatedCarries( (4*index)+3 downto (4*index)) := calculateCarries(p1, g1, calculatedCarriesTemp(index), 4*index);
   end loop;
   
   --Use the carries here to do the local addition.
   for index in 0 to 31 loop
    result(index) := calculatedCarries(index) xor a(index) xor bb(index);
   end loop;
   
   carry_out := g3(1)
                or (p3(1) and g3(0))
                or (p3(1) and p3(0) and carry_in);
   result(a'length) := carry_out xnor do_add;
   return result;
end; --function

function bv_adder(a     : in std_logic_vector;
                  b     : in std_logic_vector;
                  do_add: in std_logic) return std_logic_vector is
   variable carry_in : std_logic;
   variable bb       : std_logic_vector(a'length-1 downto 0);
   variable result   : std_logic_vector(a'length downto 0);
begin
   if do_add = '1' then
      bb := b;
      carry_in := '0';
   else
      bb := not b;
      carry_in := '1';
   end if;
   for index in 0 to a'length-1 loop
      result(index) := a(index) xor bb(index) xor carry_in;
      carry_in := (carry_in and (a(index) or bb(index))) or
                  (a(index) and bb(index));
   end loop;
   result(a'length) := carry_in xnor do_add;
   return result;
end; --function


function bv_negate(a : in std_logic_vector) return std_logic_vector is
   variable carry_in : std_logic;
   variable not_a    : std_logic_vector(a'length-1 downto 0);
   variable result   : std_logic_vector(a'length-1 downto 0);
begin
   not_a := not a;
   carry_in := '1';
   for index in a'reverse_range loop
      result(index) := not_a(index) xor carry_in;
      carry_in := carry_in and not_a(index);
   end loop;
   return result;
end; --function


function bv_increment(a : in std_logic_vector(31 downto 2)
                     ) return std_logic_vector is
   variable carry_in : std_logic;
   variable result   : std_logic_vector(31 downto 2);
begin
   carry_in := '1';
   for index in 2 to 31 loop
      result(index) := a(index) xor carry_in;
      carry_in := a(index) and carry_in;
   end loop;
   return result;
end; --function


function bv_inc(a : in std_logic_vector
                ) return std_logic_vector is
   variable carry_in : std_logic;
   variable result   : std_logic_vector(a'length-1 downto 0);
begin
   carry_in := '1';
   for index in 0 to a'length-1 loop
      result(index) := a(index) xor carry_in;
      carry_in := a(index) and carry_in;
   end loop;
   return result;
end; --function

end; --package body


