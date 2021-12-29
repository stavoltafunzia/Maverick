function readtexttable(filename)
    local check_file = io.open(filename,"r")
    if (check_file==nil) then
        error( "error, file '" .. filename .. "' does not exists")
        return nil
    else
        io.close(check_file)
        print("Lua: reading table " .. filename .. " ...")
        -- io.write("Lua: reading table " .. filename .. " ...")
        csv = require "csv"
        local f = csv.open(filename,{separator=",", header=true})

        first_line=false;
        content = {}
        for fields in f:lines() do
            if (first_line) then
                for i,v in pairs(fields) do content[i][#content[i]+1]=tonumber(v) end
            else
                first_line = true;
                for i,v in pairs(fields) do content[i]={tonumber(v)} end
            end
        end
        -- print("Done!")
        return content
    end
end
