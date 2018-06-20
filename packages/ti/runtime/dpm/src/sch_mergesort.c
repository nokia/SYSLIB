/* Bottom-up Merge Sort Algorithm */
/* The sorted is in increasing order */

#ifdef ORG_CODE
#include "sch_mergesort.h"
#else
#include <ti/runtime/dpm/include/sch_mergesort.h>
#endif


static int do_merge(unsigned int **inputs, int *sizes, unsigned int **indexes, int count, unsigned int *output, unsigned int *order) {
	int i;
	int remainder = count & 3;
	int iterations = (count - remainder)/4;
	unsigned int **current_inputs = inputs, **current_indexes = indexes;
	int *current_sizes = sizes;
	int merge_count = 0, merge_size;
	unsigned int *current_output = output, *current_order = order;
	
	if (iterations > 0 && remainder > 0 && remainder < 3) {
		remainder += 10;
		iterations--;
	}
	for (i=0;i<iterations;i++) {
		/* Do a fourWays merge.  */
		merge_size = current_sizes[0] + current_sizes[1] + current_sizes[2] + current_sizes[3];
		MergeIndex_fourWays32(current_inputs, current_sizes, current_indexes, current_output, current_order, (unsigned int *)0/*unused*/);
		inputs[i] = current_output;
		indexes[i] = current_order;
		sizes[i] = merge_size;
		merge_count++;
		current_inputs +=4;
		current_indexes +=4;
		current_sizes +=4;
		current_order += merge_size;
		current_output += merge_size;
	}
	/* Tail of the merges.  */
	switch (remainder) {
	case 11:
		/* Do one threeWays and one twoWays merge.  */
		merge_size = current_sizes[0] + current_sizes[1] + current_sizes[2];
		MergeIndex_threeWays32(current_inputs, current_sizes, current_indexes, current_output, current_order, (unsigned int *)0/*unused*/);
		inputs[merge_count] = current_output;
		indexes[merge_count] = current_order;
		sizes[merge_count] = merge_size;
		merge_count++;
		current_inputs +=3;
		current_indexes +=3;
		current_sizes +=3;
		current_order += merge_size;
		current_output += merge_size;
		/* Intentional fall-through.  */
	case 2:
		/* Here is the twoWays merge.  */ 
		merge_size = current_sizes[0] + current_sizes[1];
		MergeIndex_twoWays32(current_inputs, current_sizes, current_indexes, current_output, current_order, (unsigned int *)0/*unused*/);
		inputs[merge_count] = current_output;
		indexes[merge_count] = current_order;
		sizes[merge_count] = merge_size;
		merge_count++;
		current_inputs +=2;
		current_indexes +=2;
		current_sizes +=2;
		current_order += merge_size;
		current_output += merge_size;
		break;
	case 12:
		/* Do two threeWays merges.  */
		/* First threeWays merge.  */
		merge_size = current_sizes[0] + current_sizes[1] + current_sizes[2];
		MergeIndex_threeWays32(current_inputs, current_sizes, current_indexes, current_output, current_order, (unsigned int *)0/*unused*/);
		inputs[merge_count] = current_output;
		indexes[merge_count] = current_order;
		sizes[merge_count] = merge_size;
		merge_count++;
		current_inputs +=3;
		current_indexes +=3;
		current_sizes +=3;
		current_order += merge_size;
		current_output += merge_size;
		/* Second threeWays merge.  Fall through, no "break" statement. */
	case 3:
	case 13:
		/* Do one threeWays merge.  */
		merge_size = current_sizes[0] + current_sizes[1] + current_sizes[2];
		MergeIndex_threeWays32(current_inputs, current_sizes, current_indexes, current_output, current_order, (unsigned int *)0/*unused*/);
		inputs[merge_count] = current_output;
		indexes[merge_count] = current_order;
		sizes[merge_count] = merge_size;
		merge_count++;
		current_inputs +=3;
		current_indexes +=3;
		current_sizes +=3;
		current_order += merge_size;
		current_output += merge_size;
		break;
	case 1:
		/* There is nothing to do in this case. Why has this function been called?  */
	default:
		break;
	}
	return merge_count;
}

void SortIndex_merge32(unsigned int * restrict array, int size, unsigned int * restrict index, unsigned int * restrict output, unsigned int * restrict order, unsigned int * restrict scratch) {
	unsigned int *inputs[256], *indexes[256]; /* JJ: here the array size should be set to at least ceil(size/4) */
	int sizes[256];							  /* JJ: here the array size should be set to at least ceil(size/4) */
	unsigned int i1, i2, i3, i4;
	unsigned int idx1, idx2, idx3, idx4;
#ifdef ORG_CODE
	int insert_1st = 0, insert_2nd = 0, insert_3rd = 0, insert_4th = 0;
#else
	int insert_2nd = 0, insert_3rd = 0, insert_4th = 0;
#endif
	unsigned int i_temp, idx_temp;
	int i, remainder, iterations, list_count = 0, shift;
	unsigned int offset = 0;
	unsigned int *scratch_output, *scratch_order;

	remainder = size & 3;
	iterations = (size - remainder)/4;
	if (size>16) { offset += 16;}
	if (size>64) { offset += 64;}
	if (size>256) { offset += 256;}
	if (size>1024) { offset += 1024;} /* FIXME: For now we don't support sorting more than 4K elements.  */
	if (size <= 4) {
		scratch_output = output;
		scratch_order = order;
	} else {
		scratch_output = scratch+offset;
		scratch_order = scratch+2*offset + size + (4-size&0x3);
	}
	
	_nassert((int)array % 8 == 0);
	_nassert((int)index % 8 == 0);
	_nassert((int)scratch_output % 8 == 0);
	_nassert((int)scratch_order % 8 == 0);
	_nassert((int)output % 8 == 0);
	_nassert((int)order % 8 == 0);
	for (i=0;i<iterations;i++) {
		/* Prepare the array for the merges.  */
		indexes[i] = scratch_order+4*i;
		inputs[i] = scratch_output+4*i;
		sizes[i] = 4;
		list_count++;
		/* Sort_sortFour32(array+i, scratch+i); */
		i1 = array[4*i]; i2 = array[4*i+1];
		i3 = array[4*i+2]; i4 = array[4*i+3];
		idx1 = index[4*i]; idx2 = index[4*i+1];
		idx3 = index[4*i+2]; idx4 = index[4*i+3];
		insert_2nd = 0; insert_3rd = 0; insert_4th = 0;
		i_temp = i3; idx_temp = idx3;
		/* First element to be inserted. */
		if (i4 < i3) {
			/* We just have to swap the two inputs if required.  */
			i3 = i4; idx3 = idx4;
			i4 = i_temp; idx4 = idx_temp;
		}
		/* Second element to be inserted.  */
		i_temp = i2; idx_temp = idx2;
		if (i4 < i2) insert_3rd = 1;
		if (i3 < i2) insert_2nd = 1  & ~insert_3rd;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			i2 = i3; idx2 = idx3;
			i3 = i_temp; idx3 = idx_temp;
		}
		if (insert_3rd) {
			i2 = i3; idx2 = idx3;
			i3 = i4; idx3 = idx4;
			i4 = i_temp; idx4 = idx_temp;
		}
		/* Last element to be inserted  */
		insert_2nd = 0; insert_3rd = 0;
		i_temp = i1; idx_temp = idx1;
		if (i4 < i1) insert_4th = 1;
		if (i3 < i1) insert_3rd = 1  & ~insert_4th;
		if (i2 < i1) insert_2nd = 1  & ~insert_3rd & ~insert_4th;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			i1 = i2; idx1 = idx2;
			i2 = i_temp; idx2 = idx_temp;
		}
		if (insert_3rd) {
			i1 = i2; idx1 = idx2;
			i2 = i3; idx2 = idx3;
			i3 = i_temp; idx3 = idx_temp;
		}
		if (insert_4th) {
			i1 = i2; idx1 = idx2;
			i2 = i3; idx2 = idx3;
			i3 = i4; idx3 = idx4;
			i4 = i_temp; idx4 = idx_temp;
		}
#ifdef _TMS320C6X
		_amem8((void *)&(scratch_output[4*i])) = _itoll(i2, i1);
		_amem8((void *)&(scratch_output[4*i+2])) = _itoll(i4, i3);
		_amem8((void *)&(scratch_order[4*i])) = _itoll(idx2, idx1);
		_amem8((void *)&(scratch_order[4*i+2])) = _itoll(idx4, idx3);
#else
		scratch_output[4*i] = i1; scratch_output[4*i+1] = i2;
		scratch_output[4*i+2] = i3; scratch_output[4*i+3] = i4;
		scratch_order[4*i] = idx1; scratch_order[4*i+1] = idx2;
		scratch_order[4*i+2] = idx3; scratch_order[4*i+3] = idx4;
#endif
	}
	/* Merge the remaining elements if any.  */
	if (remainder > 0) {
		indexes[i] = scratch_order+4*i;
		inputs[i] = scratch_output+4*i;
		sizes[i] = remainder;
		list_count++;
	}
	switch(remainder) {
	case 0:
		break;
	case 1:
		scratch_output[4*i] = array[4*i];
		scratch_order[4*i] = index[4*i];
		break;
	case 2:
		i1 = array[4*i]; i2 = array[4*i+1];
		idx1 = index[4*i]; idx2 = index[4*i+1];
		if (i1 <= i2) {
			scratch_output[4*i] = i1; scratch_output[4*i+1] = i2;
			scratch_order[4*i] = idx1; scratch_order[4*i+1] = idx2;
		} else {
			scratch_output[4*i] = i2; scratch_output[4*i+1] = i1;
			scratch_order[4*i] = idx2; scratch_order[4*i+1] = idx1;
		}
		break;
	case 3:
		i1 = array[4*i]; i2 = array[4*i+1]; i3 = array[4*i+2];
		idx1 = index[4*i]; idx2 = index[4*i+1]; idx3 = index[4*i+2];
		/* Sort the three entries.  */
		if (i2 < i1) {
			i_temp = i1; idx_temp = idx1;
			i1 = i2; idx1 = idx2;
			i2 = i_temp; idx2 = idx_temp;
		}
		if (i3 < i2) {
			i_temp = i2; idx_temp = idx2;
			i2 = i3; idx2 = idx3;
			i3 = i_temp; idx3 = idx_temp;
		}
		if (i2 < i1) {
			i_temp = i1; idx_temp = idx1;
			i1 = i2; idx1 = idx2;
			i2 = i_temp; idx2 = idx_temp;
		}
		/* Write the output.  */
		scratch_output[4*i] = i1; scratch_output[4*i+1] = i2; scratch_output[4*i+2] = i3;
		scratch_order[4*i] = idx1; scratch_order[4*i+1] = idx2; scratch_order[4*i+2] = idx3;
		break;
	}
	if (list_count < 2) return;
	/* All array elements are now sorted by groups of 4.  We are about to start
	   merging all the array elements.  */
	shift = 16;
	while (list_count > 4) {
		/* We merge into a temporary space.  */
		scratch_output -= shift;
		scratch_order -= shift;
		shift *= 4;
		list_count = do_merge(inputs, sizes, indexes, list_count, scratch_output, scratch_order);
	}
	/* This is the final merge that will produce the final output.  */
	do_merge(inputs, sizes, indexes, list_count, output, order);
}



/*******************************************/
/*** Merge Functions with Index Ordering ***/
/*******************************************/

/* Two-Way Merge */
void MergeIndex_twoWays32 (unsigned int **inputs, int *sizes, unsigned int **indexes, \
						   unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch) {
	/* In this optimized version we are using restricts and replacing the original loop
	   which had a complex condition in it with two loops.  */
	int i;
	int count1 = 0, count2 = 0;
	unsigned int * restrict in1 = inputs[0], * restrict in2 = inputs[1];
	unsigned int * restrict ord1 = indexes[0], * restrict ord2 = indexes[1];
	int size1 = sizes[0], size2 = sizes[1];
	unsigned int val1 , val2;
	
	_nassert((int)in1 % 8 == 0);
	_nassert((int)in2 % 8 == 0);
	_nassert((int)out % 8 == 0);
	_nassert((int)scratch % 8 == 0);
	if (in1[size1-1] < in2[size2-1]) {
		/* Just swap the two lists. */
		unsigned int *in_temp = in1, *ord_temp = ord1;
		int size_temp = size1;
		in1 = in2; in2 = in_temp;
		size1 = size2; size2 = size_temp;
		ord1 = ord2; ord2 = ord_temp;
	}
	val1 = *in1; val2 = *in2;
	for (i=0; count2 < size2; i++) {
		/* Merge until 'in2' is exhausted first.  */
		if (val1 < val2) {
			*out++ = val1;
			val1 = *++in1;
			*order++ = *ord1++;
			count1++;
		} else {
			*out++ = val2;
			val2 = *++in2;
			*order++ = *ord2++;
			count2++;
		}
	}
	for (; count1 < size1; i++) {
		/* Just copy the remaining elements in 'in1' to 'out'.  */
		*out++ = *in1++;
		*order++ = *ord1++;
		count1++;
	}
}

/* Three-Way Merge */
void MergeIndex_threeWays32(unsigned int **inputs, int *sizes, unsigned int **indexes, unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch) {
	unsigned int * restrict in1 = inputs[0], * restrict in2 = inputs[1], * restrict in3 = inputs[2];
	int size1 = sizes[0], size2 = sizes[1], size3 = sizes[2];
	int count1 = 0, count2 = 0, count3 = 0;
	unsigned int value1 = *in1, value2 = *in2, value3 = *in3;
	unsigned int * restrict ord1 = indexes[0], * restrict ord2 = indexes[1], * restrict ord3 = indexes[2];
	int insert_2nd = 0, insert_3rd = 0;
	unsigned int *in_temp, *ord_temp;
	unsigned int value_temp;
	int size_temp, count_temp;

	/* As the merge is progressing the various inputs are exhausted and the code deals differently
	   with each stage. For the 'Four inputs' and 'Three inputs' stages we basically keep a 'list'
	   of the inputs sorted (ascending) by the value of their current first value.  */

	/* PHASE #0: Sort the inputs one first time, sort is ascending.  */
	{
		insert_2nd = 0; insert_3rd = 0;
		in_temp = in2; ord_temp = ord2;
		value_temp = value2;
		size_temp = size2;
		/* First element to be inserted. */
		if (value3 < value2) {
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		/* Second element to be inserted.  */
		in_temp = in1; ord_temp = ord1;
		value_temp = value1;
		size_temp = size1;
		if (value3 < value1) insert_3rd = 1;
		if (value2 < value1) insert_2nd = 1  & ~insert_3rd;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
	}
	
	/* PHASE #1: Three inputs processing.  */
	/* The list of inputs is already sorted the first time.  */
	*out++ = value1;
	value1 = *++in1;
	*order++ = *ord1++;
	count1++;
	for (;count1<size1;) {
		insert_2nd = 0; insert_3rd = 0;
		/* Sort the inputs.  The nice thing is that we just have insert 'value1' in
		   the right position because the rest of the list (2 other elements) is
		   already sorted.  */
		if (value3 < value1) insert_3rd = 1;
		if (value2 < value1) insert_2nd = 1  & ~insert_3rd;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count_temp; size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			count3 = count_temp; size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		/* Add the first element of 'in1' to the 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
	/* Get rid of the first input, and shift the other two into position.  */
	count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
	count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
	
	/* PHASE #2: Two inputs processing.  */
	/* The list of inputs is already sorted the first time.  */
	*out++ = value1;
	value1 = *++in1;
	*order++ = *ord1++;
	count1++;
	for (;count1<size1;) {
		/* Sort the inputs.  The nice thing is that we just have insert 'value1' in
		   the right position because the rest of the list (1 other element) is already sorted.  */
		if (value2 < value1) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count_temp; size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		/* Add the first element of 'in1' to the 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
	/* Get rid of the first input, and shift the other two into position.  */
	count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;

	/* PHASE #3: One input left. */
	for (; count1 < size1; ) {
		/* Just copy the remaining elements in 'in1' to 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
}


/* Four-Way Merge */
void MergeIndex_fourWays32(unsigned int **inputs, int *sizes, unsigned int **indexes, unsigned int * restrict out, unsigned int * restrict order, unsigned int * restrict scratch) {
	unsigned int * restrict in1 = inputs[0], * restrict in2 = inputs[1], * restrict in3 = inputs[2], * restrict in4 = inputs[3];
	int size1 = sizes[0], size2 = sizes[1], size3 = sizes[2], size4 = sizes[3];
	int count1 = 0, count2 = 0, count3 = 0, count4 = 0;
	unsigned int value1 = *in1, value2 = *in2, value3 = *in3, value4 = *in4;
	unsigned int * restrict ord1 = indexes[0], * restrict ord2 = indexes[1], * restrict ord3 = indexes[2], * restrict ord4 = indexes[3];
	int insert_2nd = 0, insert_3rd = 0, insert_4th = 0;
	unsigned int *in_temp, *ord_temp;
	unsigned int value_temp;
	int size_temp, count_temp;

	/* As the merge is progressing the various inputs are exhausted and the code deals differently
	   with each stage. For the 'Four inputs' and 'Three inputs' stages we basically keep a 'list'
	   of the inputs sorted (ascending) by the value of their current first value.  */

	/* PHASE #0: Sort the inputs one first time, sort is ascending.  */
	{
		insert_2nd = 0; insert_3rd = 0; insert_4th = 0;
		in_temp = in3; ord_temp = ord3;
		value_temp = value3;
		size_temp = size3;
		/* First element to be inserted. */
		if (value4 < value3) {
			size3 = size4; in3 = in4; value3 = value4; ord3 = ord4;
			size4 = size_temp; in4 = in_temp; value4 = value_temp; ord4 = ord_temp;
		}
		/* Second element to be inserted.  */
		in_temp = in2; ord_temp = ord2;
		value_temp = value2;
		size_temp = size2;
		if (value4 < value2) insert_3rd = 1;
		if (value3 < value2) insert_2nd = 1  & ~insert_3rd;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size4; in3 = in4; value3 = value4; ord3 = ord4;
			size4 = size_temp; in4 = in_temp; value4 = value_temp; ord4 = ord_temp;
		}
		/* Last element to be inserted  */
		insert_2nd = 0; insert_3rd = 0; insert_4th = 0;
		in_temp = in1; ord_temp = ord1;
		value_temp = value1;
		size_temp = size1;
		if (value4 < value1) insert_4th = 1;
		if (value3 < value1) insert_3rd = 1  & ~insert_4th;
		if (value2 < value1) insert_2nd = 1  & ~insert_3rd & ~insert_4th;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		if (insert_4th) {
			/* We just have to swap the two inputs if required.  */
			size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			size3 = size4; in3 = in4; value3 = value4; ord3 = ord4;
			size4 = size_temp; in4 = in_temp; value4 = value_temp; ord4 = ord_temp;
		}
	}
	
        /* PHASE #1: Four inputs processing.  */
	/* The list of inputs is already sorted the first time.  */
	*out++ = value1;
	value1 = *++in1;
	*order++ = *ord1++;
	count1++;
	for (;count1<size1;) {
		insert_2nd = 0; insert_3rd = 0; insert_4th = 0;
		/* Sort the inputs.  The good thing is that we just have insert 'value1' in
		   the right position because the rest of the list (3 other elements) is
		   already sorted.  */
		if (value4 < value1) insert_4th = 1;
		if (value3 < value1) insert_3rd = 1  & ~insert_4th;
		if (value2 < value1) insert_2nd = 1  & ~insert_3rd & ~insert_4th;
		in_temp = in1; ord_temp = ord1;
		value_temp = value1;
		count_temp = count1; size_temp = size1;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count_temp; size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			count3 = count_temp; size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		if (insert_4th) {
			/* We just have to swap the two inputs if required.  */
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			count3 = count4; size3 = size4; in3 = in4; value3 = value4; ord3 = ord4;
			count4 = count_temp; size4 = size_temp; in4 = in_temp; value4 = value_temp; ord4 = ord_temp;
		}
		/* Add the first element of 'in1' to the 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
	/* Get rid of the first input, and shift the other three into position.  */
	count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
	count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
	count3 = count4; size3 = size4; in3 = in4; value3 = value4; ord3 = ord4;
	
	/* PHASE #2: Three inputs processing.  */
	/* The list of inputs is already sorted the first time.  */
	*out++ = value1;
	value1 = *++in1;
	*order++ = *ord1++;
	count1++;
	for (;count1<size1;) {
		insert_2nd = 0; insert_3rd = 0;
		/* Sort the inputs.  The nice thing is that we just have insert 'value1' in
		   the right position because the rest of the list (2 other elements) is
		   already sorted.  */
		if (value3 < value1) insert_3rd = 1;
		if (value2 < value1) insert_2nd = 1  & ~insert_3rd;
		if (insert_2nd) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count_temp; size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		if (insert_3rd) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
			count3 = count_temp; size3 = size_temp; in3 = in_temp; value3 = value_temp; ord3 = ord_temp;
		}
		/* Add the first element of 'in1' to the 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
	/* Get rid of the first input, and shift the other two into position.  */
	count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
	count2 = count3; size2 = size3; in2 = in3; value2 = value3; ord2 = ord3;
	
	/* PHASE #3: Two inputs processing.  */
	/* The list of inputs is already sorted the first time.  */
	*out++ = value1;
	value1 = *++in1;
	*order++ = *ord1++;
	count1++;
	for (;count1<size1;) {
		/* Sort the inputs.  The nice thing is that we just have insert 'value1' in
		   the right position because the rest of the list (1 other element) is already sorted.  */
		if (value2 < value1) {
			/* We just have to swap the two inputs if required.  */
			in_temp = in1; ord_temp = ord1;
			value_temp = value1;
			count_temp = count1; size_temp = size1;
			count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;
			count2 = count_temp; size2 = size_temp; in2 = in_temp; value2 = value_temp; ord2 = ord_temp;
		}
		/* Add the first element of 'in1' to the 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
	/* Get rid of the first input, and shift the other two into position.  */
	count1 = count2; size1 = size2; in1 = in2; value1 = value2; ord1 = ord2;

	/* PHASE #4: One input left. */
	for (; count1 < size1; ) {
		/* Just copy the remaining elements in 'in1' to 'out'.  */
		*out++ = value1;
		value1 = *++in1;
		*order++ = *ord1++;
		count1++;
	}
}


