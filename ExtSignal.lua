--[[
Copyright Extrarius 2024
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is a very primtive type-safe signal implementation.
]]
--!strict
--!optimize 2
local ExtSignal = {}

export type Connection<T...> = {
    Disconnect: (conn: Connection<T...>) -> (),
    Connected: boolean,
    _Signal: Signal<T...>,
    _Callback: (T...) -> ()
}

export type Signal<T...> = {
    Connect: (Signal<T...>, (T...) -> ()) -> Connection<T...>,
    Fire: (sig: Signal<T...>, T...) -> (),
    DisconnectAll: (sig: Signal<T...>) -> (),
    _Connections: {[Connection<T...>]: true},
}

--Basic system for keeping a pool of threads that are used to run callbacks
local threadPool: {thread} = {}
local function RunThreadCallback(callback, ...)
    callback(...)
    table.insert(threadPool, coroutine.running())
end
local function ThreadPoolFunc()
    while true do
        RunThreadCallback(coroutine.yield())
    end
end


--Disconnect a connection so it will no longer be called when the signal is fired
local function Disconnect<T...>(conn: Connection<T...>)
    conn.Connected = false
    conn._Signal._Connections[conn] = nil
end
ExtSignal.Disconnect = Disconnect


--Connect a callback to a signal and return a connection that can be used to disconnect it
local function Connect<T...>(sig: Signal<T...>, callback: (T...) -> ()): Connection<T...>
    local conn: Connection<T...> = {
        Disconnect = Disconnect,
        Connected = true,
        _Signal = sig,
        _Callback = callback
    }
    sig._Connections[conn] = true
    return conn
end
ExtSignal.Connect = Connect


--Fire all connected callbacks with the given arguments
local function Fire<T...>(sig: Signal<T...>, ...: T...)
    local conns = table.clone(sig._Connections)
    for conn, _ in conns do
        local fireThread = table.remove(threadPool)
        if fireThread == nil then
            fireThread = coroutine.create(ThreadPoolFunc)
            coroutine.resume(fireThread :: thread)
        end
        local fireThread = fireThread :: thread --type system hack to turn "thread?" into "thread" from here on
        task.spawn(fireThread, conn._Callback, ...)
     end
end
ExtSignal.Fire = Fire


--Disconnect all connections for a given signal
local function DisconnectAll<T...>(sig: Signal<T...>)
    local conns = table.clone(sig._Connections)
    for conn, _ in conns do
        Disconnect(conn)
    end
end
ExtSignal.DisconnectAll = DisconnectAll


--Create a new signal
local function NewSignal<T...>(): Signal<T...>
    return {
        Connect = Connect,
        Fire = Fire,
        DisconnectAll = DisconnectAll,
        _Connections = {}
    }
end
ExtSignal.NewSignal = NewSignal


--
return table.freeze(ExtSignal)