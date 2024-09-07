
%_pb2.py: %.proto
	protoc --python_out=$(dir $@) $<
