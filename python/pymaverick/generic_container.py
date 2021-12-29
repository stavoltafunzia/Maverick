import pymaverick.core as mvc


class GenericContainer:

    def __init__(self, data: dict) -> None:
        self._gc_id = mvc.convert_dict_to_gc(data, False)
        mvc.convert_dict_to_gc(data, print_content=False)

    def __del__(self) -> None:
        mvc.delete_gc(self._gc_id)

    def get_mem_ptr(self) -> int:
        gc_mem_ptr = mvc.get_gc_memptr(self._gc_id)
        if gc_mem_ptr is None:
            raise RuntimeError("Null pointer to input data GenericContainer")
        return gc_mem_ptr

    @staticmethod
    def from_c_container(gc_id, delete=True) -> dict:
        mvc.check_init()
        # extract the data from the solution
        out_data = mvc.get_gc_content(gc_id)
        # clean the current generic container
        if delete:
            mvc.delete_gc(gc_id)

        return out_data
