library verilog;
use verilog.vl_types.all;
entity hazard_detection is
    port(
        MEM_rt          : in     vl_logic_vector(4 downto 0);
        EX_rt           : in     vl_logic_vector(4 downto 0);
        ID_rs           : in     vl_logic_vector(4 downto 0);
        ID_rt           : in     vl_logic_vector(4 downto 0);
        EX_memread      : in     vl_logic;
        MEM_memread     : in     vl_logic;
        jump            : in     vl_logic;
        pcsrc           : in     vl_logic;
        jumpreg         : in     vl_logic;
        mux_c           : out    vl_logic;
        pc_c            : out    vl_logic;
        IF_ID_c         : out    vl_logic
    );
end hazard_detection;
