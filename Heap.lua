--[[
Copyright Extrarius 2023.
This work is marked with CC0 1.0 Universal.
To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0
--
This is an implementation of a pairing max-heap, which has constant time insertion.
Algorithms implemented from https://en.wikipedia.org/wiki/Pairing_heap
The algorithms were modified to create fewer temporaries, thus reducing garbage generated.
]]

--!strict

--The base node type used to build the heap
type PairingHeapNode = {value: any?, priority: number, subheaps: {PairingHeapNode}}

--The type this module creates
type HeapImpl = {
	__index: HeapImpl,
	new: () -> Heap,
	push: (self: Heap, priority: number, value: any) -> (),
	peek: (self: Heap) -> (any, number),
	pop: (self: Heap) -> (any, number),
	_Test: () -> ()
}
type Heap = typeof(setmetatable({} :: PairingHeapNode, {} :: HeapImpl))

local Heap: HeapImpl = {} :: HeapImpl
Heap.__index = Heap

--Construct a new empty Heap
function Heap.new(): Heap
	return setmetatable({value = nil, priority = -math.huge, subheaps = {}} :: PairingHeapNode, Heap)
end

--Insert an element
function Heap:push(priority: number, value: any)
	if self.value == nil then
		self.value = value
		self.priority = priority
		self.subheaps = {}
	elseif self.priority > priority then
		table.insert(self.subheaps, {value = value, priority = priority, subheaps = {}} :: PairingHeapNode)
	else
		table.insert(self.subheaps, {value = self.value, priority = self.priority, subheaps = {}} :: PairingHeapNode)
		self.value = value
		self.priority = priority
	end
end

--Peek at the highest priority element
function Heap:peek(): (any, number)
	return self.value, self.priority
end

--Utility function used by pop to (destructively) meld two heaps into one new heap
local function _Meld<T>(heap1: PairingHeapNode, heap2: PairingHeapNode): PairingHeapNode
	--print("Melding (", heap1.value, " @ ", heap1.priority, ") with (", heap2.value, " @ ", heap2.priority, ")")
	if heap1.value == nil then
		return heap2
	elseif heap2.value == nil then
		return heap1
	elseif heap1.priority > heap2.priority then
		table.insert(heap1.subheaps, heap2)
		return heap1
	else
		table.insert(heap2.subheaps, heap1)
		return heap2
	end
end

--Utility function used by pop to (destructively) remerge sub-heaps
local function _MergePairs(list: {PairingHeapNode}): PairingHeapNode
	while #list > 1 do
		local results = table.create((#list + 1) // 2)
		local numresults = (#list + 1) // 2
		for ii = 1,#list-1,2 do
			results[(ii + 1) // 2] = _Meld(list[ii], list[ii+1])
		end
		if #list % 2 == 1 then
			results[numresults] = list[#list]
		end
		list = results
	end
	return list[1]
end

--Remove the highest priority element and return the value and prority
function Heap:pop(): (any, number)
	local value = self.value
	local priority = self.priority
	--print("Popping value ", value)
	if (self.value ~= nil) and (#self.subheaps > 0) then
		local heap = _MergePairs(self.subheaps)
		self.value = heap.value
		self.priority = heap.priority
		self.subheaps = heap.subheaps
	else
		self.value = nil
		self.priority = -math.huge
		self.subheaps = {}
	end
	return value, priority
end

function Heap._Test()
	local h = Heap.new()

	--Test popping empty is nil
	assert(h:pop() == nil)

	--Test next most basic scenario, 
	h:push(0, "A")
	assert(h:pop() == "A")

	--All permutations of 2-element priorities
	h:push(0, "A")
	h:push(1, "B")
	assert(h:pop() == "B")
	assert(h:pop() == "A")

	h:push(1, "A")
	h:push(0, "B")
	assert(h:pop() == "A")
	assert(h:pop() == "B")

	--All permutations of 3-element priorities
	h:push(0, "A")
	h:push(1, "B")
	h:push(2, "C")
	assert(h:pop() == "C")
	assert(h:pop() == "B")
	assert(h:pop() == "A")

	h:push(0, "A")
	h:push(2, "B")
	h:push(1, "C")
	assert(h:pop() == "B")
	assert(h:pop() == "C")
	assert(h:pop() == "A")

	h:push(1, "A")
	h:push(0, "B")
	h:push(2, "C")
	assert(h:pop() == "C")
	assert(h:pop() == "A")
	assert(h:pop() == "B")

	h:push(2, "A")
	h:push(0, "B")
	h:push(1, "C")
	assert(h:pop() == "A")
	assert(h:pop() == "C")
	assert(h:pop() == "B")

	h:push(1, "A")
	h:push(2, "B")
	h:push(0, "C")
	assert(h:pop() == "B")
	assert(h:pop() == "A")
	assert(h:pop() == "C")

	h:push(2, "A")
	h:push(1, "B")
	h:push(0, "C")
	assert(h:pop() == "A")
	assert(h:pop() == "B")
	assert(h:pop() == "C")

	--Test pushing more elements out of order
	h:push(10, "A")
	h:push(5, "B")
	h:push(7, "C")
	h:push(15, "D")
	h:push(1, "E")
	assert(h:pop() == "D")
	assert(h:pop() == "A")
	assert(h:pop() == "C")
	assert(h:pop() == "B")
	assert(h:pop() == "E")

	--Test pushing non-empty heap
	h:push(10, "A")
	h:push(5, "B")
	h:push(7, "C")
	h:push(15, "D")
	assert(h:pop() == "D")
	assert(h:pop() == "A")
	h:push(6, "E")
	h:push(1, "F")
	h:push(20, "G")
	assert(h:pop() == "G")
	assert(h:pop() == "C")
	assert(h:pop() == "E")
	assert(h:pop() == "B")
	assert(h:pop() == "F")

	--Test popping empty is still nil
	assert(h:pop() == nil)

	--Test interspersing pops and pushes on a larger value set (values 100-199)
	local TestValues: {number} = {
		112,170,166,151,180,136,145,105,138,176,135,120,148,194,161,125,143,164,189,171,
		169,104,103,153,130,122,162,117,121,137,126,152,149,116,146,150,177,178,191,119,
		127,133,159,179,157,186,185,192,183,111,158,197,196,155,174,109,128,160,106,165,
		199,141,190,142,188,175,110,173,107,195,163,193,167,147,132,123,139,129,156,172,
		198,114,118,100,102,140,184,182,134,113,144,101,187,108,115,131,168,124,154,181
	}

	local TestTracker: {number} = {}

	local function SubArray(arr, start, finish)
		return {unpack(arr, start, finish)}
	end

	local function MultiPush(start: number, finish: number)
		for _, val in ipairs(SubArray(TestValues, start, finish)) do
			h:push(val, val)
			table.insert(TestTracker, val)
		end
	end

	local function MultiPop(count: number)
		table.sort(TestTracker, function(a: number, b: number): boolean return a > b end)
		for i = 1, count do
			assert(h:pop() == TestTracker[i])
		end
		local temp = SubArray(TestTracker, count + 1, #TestTracker)
		TestTracker = temp
	end

	MultiPush(1, 10)
	MultiPop(1)
	MultiPush(11, 20)
	MultiPop(2)
	MultiPush(21, 30)
	MultiPop(3)
	MultiPush(31, 40)
	MultiPop(5)
	MultiPush(41, 50)
	MultiPop(7)
	MultiPush(51, 60)
	MultiPop(11)
	MultiPush(61, 70)
	MultiPop(13)
	MultiPush(71, 80)
	MultiPop(17)
	MultiPush(81, 90)
	MultiPop(19)
	MultiPush(91, 100)
	MultiPop(#TestTracker)

	--Test popping empty is still nil
	assert(h:pop() == nil)

	--Test completed
	print("Heap test succeeded")
end


return Heap
