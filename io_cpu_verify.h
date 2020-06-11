/*
 *
 * tests specific to the same70 CPU
 *
 */
#ifndef io_cpu_verify_H_
#define io_cpu_verify_H_
#ifdef IMPLEMENT_VERIFY_IO_CPU

TEST_BEGIN(test_io_random_1) {
	uint32_t rand[3],count;

/*	rand[0] = io_get_random_u32(TEST_IO);
	rand[1] = io_get_random_u32(TEST_IO);
	rand[2] = io_get_random_u32(TEST_IO);


	if (rand[0] == rand[1])	rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);

	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);
*/
	rand[0] = io_get_next_prbs_u32(TEST_IO);
	rand[1] = io_get_next_prbs_u32(TEST_IO);
	rand[2] = io_get_next_prbs_u32(TEST_IO);

	count = 100;
	do {
		if (rand[0] == rand[1])	{
			rand[1] = io_get_next_prbs_u32(TEST_IO);
		} else {
			break;
		}
	} while (--count);
	
	count = 100;
	do {
		if (rand[1] == rand[2])	{
			rand[2] = io_get_next_prbs_u32(TEST_IO);
		} else {
			break;
		}
	} while (--count);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);
}
TEST_END

static void
test_io_events_1_ev (io_event_t *ev) {
	*((uint32_t*) ev->user_value) = 1;
}

TEST_BEGIN(test_io_events_1) {
	volatile uint32_t a = 0;
	io_event_t ev;
		
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);

	io_enqueue_event (TEST_IO,&ev);
	while (a == 0);
	VERIFY (a == 1,NULL);
}
TEST_END

TEST_BEGIN(test_time_clock_alarms_1) {
	volatile uint32_t a = 0;
	io_alarm_t alarm;
	io_event_t ev;
	io_time_t t;
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);
	
	t = io_get_time (TEST_IO);
	initialise_io_alarm (
		&alarm,&ev,&ev,
		(io_time_t) {t.nanoseconds + millisecond_time(200).nanoseconds}
	);

	io_enqueue_alarm (TEST_IO,&alarm);
	
	while (a == 0);
	VERIFY (a == 1,NULL);	
}
TEST_END

static void
test_io_backroom_1_entry (io_backroom_t *p) {
	(*((int*) p->user_value))++;
}

TEST_BEGIN(test_io_backroom_1) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *br;
	int r = 0;
	
	io_byte_memory_get_info (bm,&begin);

	br = create_io_backroom (TEST_IO,test_io_backroom_1_entry,&r,256);
	VERIFY (switch_to_backroom (br) && r == 1,NULL);

	free_io_backroom (br);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

static void
test_io_backroom_2_entry (io_backroom_t *p) {
	int *c = p->user_value;
	
	while (*c < 2) {
		*c = *c + 1;
		yield_to_main (p);
	}
}

TEST_BEGIN(test_io_backroom_2) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *br;
	int r = 0;
	
	io_byte_memory_get_info (bm,&begin);

	br = create_io_backroom (TEST_IO,test_io_backroom_2_entry,&r,256);
	while (switch_to_backroom (br));
	
	VERIFY (r == 2,NULL);

	free_io_backroom (br);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

static void
test_io_backroom_3_entry (io_backroom_t *br) {
	int *c = br->user_value;
	
	while (1) {
		*c = *c + 1;
		if (*c >= 2) break;
		yield_to_main (br);
	};
}

TEST_BEGIN(test_io_backroom_3) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;
	io_backroom_t *p1,*p2;
	int r = 0, s = 1;
	
	io_byte_memory_get_info (bm,&begin);

	p1 = create_io_backroom (TEST_IO,test_io_backroom_3_entry,&r,256);
	p2 = create_io_backroom (TEST_IO,test_io_backroom_3_entry,&s,256);

	VERIFY (switch_to_backroom (p2),NULL);
	VERIFY (r == 0 && s == 2,NULL);
	VERIFY (switch_to_backroom (p1),NULL);
	VERIFY (r == 1 && s == 2,NULL);
	
	VERIFY (switch_to_backroom (p1),NULL);
	
	VERIFY (s == 2 && r == 2,NULL);
	VERIFY (!switch_to_backroom (p2),NULL);
	VERIFY (s == 2 && r == 2,NULL);

	VERIFY (!switch_to_backroom (p1),NULL);
	VERIFY (!switch_to_backroom (p2),NULL);

	free_io_backroom (p1);
	free_io_backroom (p2);

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

UNIT_SETUP(setup_io_cpu_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_io_cpu_unit_test) {
}

void
io_cpu_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		test_io_events_1,
		test_time_clock_alarms_1,
#if 1
		test_io_backroom_1,
		test_io_backroom_2,
		test_io_backroom_3,
#endif
		0
	};
	unit->name = "io cpu";
	unit->description = "io cpu unit test";
	unit->tests = tests;
	unit->setup = setup_io_cpu_unit_test;
	unit->teardown = teardown_io_cpu_unit_test;
}

void
run_ut_io_cpu (V_runner_t *runner) {
	static const unit_test_t test_set[] = {
		io_cpu_unit_test,
		0
	};
	V_run_unit_tests(runner,test_set);
}

#endif /* IMPLEMENT_VERIFY_IO_CPU */
#endif
/*
MIT License
Copyright (c) 2020 Gregor Bruce
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
