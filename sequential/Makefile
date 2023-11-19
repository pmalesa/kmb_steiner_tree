CC=gcc

CFLAGS=-Wall -Werror -std=c11

TARGET=main
OBJDIR=obj
SRCDIR=src

# Add the object files for the new source files
OBJ=$(OBJDIR)/$(TARGET).o $(OBJDIR)/graph.o $(OBJDIR)/kmb.o

# The all target should depend on the TARGET
all: $(OBJDIR) $(TARGET)

# The main target depends on the object files
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJ)

# Pattern rule for building object files
$(OBJDIR)/%.o: $(SRCDIR)/%.c $(SRCDIR)/%.h
	$(CC) $(CFLAGS) -c $< -o $@

# Special rule for main, as it may not have a corresponding header file
$(OBJDIR)/$(TARGET).o: $(TARGET).c
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to ensure the object directory exists
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Clean rule
clean:
	rm -f $(TARGET) $(OBJ)
	rm -rf $(OBJDIR)

