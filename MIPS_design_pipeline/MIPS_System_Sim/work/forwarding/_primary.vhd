library verilog;
use verilog.vl_types.all;
entity forwarding is
    port(
        ID_rs           : in     vl_logic_vector(4 downto 0);
        ID_rt           : in     vl_logic_vector(4 downto 0);
        EX_rs           : in     vl_logic_vector(4 downto 0);
        EX_rt           : in     vl_logic_vector(4 downto 0);
        MEM_writereg    : in     vl_logic_vector(4 downto 0);
        WB_writereg     : in     vl_logic_vector(4 downto 0);
        MEM_regwrite    : in     vl_logic;
        WB_regwrite     : in     vl_logic;
        mux1c           : out    vl_logic_vector(1 downto 0);
        mux2c           : out    vl_logic_vector(1 downto 0);
        mux3c           : out    vl_logic_vector(1 downto 0);
        mux4c           : out    vl_logic_vector(1 downto 0)
    );
end forwarding;
