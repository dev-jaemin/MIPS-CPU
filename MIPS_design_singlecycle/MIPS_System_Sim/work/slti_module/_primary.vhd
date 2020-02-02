library verilog;
use verilog.vl_types.all;
entity slti_module is
    port(
        op              : in     vl_logic_vector(5 downto 0);
        aluout          : in     vl_logic_vector(31 downto 0);
        \out\           : out    vl_logic_vector(31 downto 0)
    );
end slti_module;
