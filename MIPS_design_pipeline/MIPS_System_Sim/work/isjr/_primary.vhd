library verilog;
use verilog.vl_types.all;
entity isjr is
    port(
        op              : in     vl_logic_vector(5 downto 0);
        funct           : in     vl_logic_vector(5 downto 0);
        d               : out    vl_logic
    );
end isjr;
