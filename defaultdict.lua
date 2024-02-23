--[[
Copyright Extrarius 2024
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is a basic implementation of python's defaultdict type, which inserts a default value when an unset key is read.
The default value is specified by a function that returns the desired value. This allows more freedom.
As a shortcut, the defaultdict will wrap non-function defaults for you. For default values that are tables,
the generated function will call table.clone to ensure each index gets a separate table. For other types you might want
cloned, provide a default function instead of a value.
]]
--!strict
--!optimize 2
type anytable = {[any]: any}
type defaultfunc = () -> any
export type defaultdict = typeof(setmetatable({} :: anytable, {} :: {__index: (anytable, any) -> any}))


local function New(default: defaultfunc | any): defaultdict
    --Make things a little easier by accepting default values, not just a generator function
    if type(default) ~= "function" then
        local value = default

        if type(value) == "table" then
            --tables should definitely probably be cloned instead of everything point to a single copy
            default = function()
                return table.clone(value)
            end
        else
            default = function()
                return value
            end
        end
    end

    local function Index(table: anytable, key: any): any
        local value = (default :: defaultfunc)()
        table[key] = value
        return value
    end

    return setmetatable({} :: anytable, {__index = Index})
end

return New