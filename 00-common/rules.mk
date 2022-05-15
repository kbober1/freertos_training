.phony: all clean

all: $(FINAL_TARGETS)

.PRECIOUS: $(OUT_DIR)/. $(OUT_DIR)%/.

$(OUT_DIR)/.:
	@$(MK) -p $@

$(OUT_DIR)%/.:
	@$(MK) -p $@

.SECONDEXPANSION:

ifeq ($(TARGET_ARCH_NAME),linux)

$(TARGET): $(OBJS)
	@echo "(LD) $@"
	@$(CC) $^ -o $@ $(LDFLAGS)
	@$(SIZE) $@

else ifeq ($(TARGET_ARCH_NAME),stm32f0)

$(TARGET).hex: $(TARGET).elf
	@$(OBJCOPY) -O ihex $^ $@
	@$(SIZE) $^

$(TARGET).elf: $(OBJS)
	@echo "(LD) $@"
	$(CC) $^ -o $@ $(LDFLAGS)

flash: $(TARGET).hex
	@st-flash --reset --format ihex write $(TARGET).hex

else

$(error Not supported TARGET_ARCH_NAME)

endif
	
$(OUT_DIR)/main/%.o: $(PROJECT_DIR)/%.c | $$(@D)/.
	@echo "(CC) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

$(OUT_DIR)/main/%.o: $(PROJECT_DIR)/%.s | $$(@D)/.
	@echo "(ASM) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

$(OUT_DIR)/freertos/%.o: $(FREERTOS_SRC_DIR)/%.c | $$(@D)/.
	@echo "(CC) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

$(OUT_DIR)/freertos/%.s: $(FREERTOS_SRC_DIR)/%.s | $$(@D)/.
	@echo "(ASM) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

$(OUT_DIR)/common/%.o: $(COMMON_SRC_DIR)/%.c | $$(@D)/.
	@echo "(CC) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

$(OUT_DIR)/common/%.o: $(COMMON_SRC_DIR)/%.s | $$(@D)/.
	@echo "(ASM) $@"
	@$(CC) -c $(CFLAGS) $< -o $@

clean:
	@$(RM) -rf $(OUT_DIR)
